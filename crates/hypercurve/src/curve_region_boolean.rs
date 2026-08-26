//! Immediate exact Booleans over curved regions.

use std::cmp::Ordering;
use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

use crate::bezier_moment::RationalQuadraticAreaIntegralCache;
use crate::bezier_offset::{
    BezierAlgebraicChordAxisDirection2, BezierAlgebraicChordPairIntersections2,
    BezierAlgebraicChordParallelIntersections2, BezierAlgebraicChordRationalIntersections2,
    BezierAlgebraicChordRationalOverlap2, BezierAlgebraicCuspSemicircleRetainedChordContact2,
    BezierAlgebraicCuspSemicircleRetainedChordIntersections2,
    BezierAlgebraicCuspSemicircleSelectedFiberContact2,
    BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2,
};
use crate::bezier_offset::{
    BezierAlgebraicCuspSemicircleContactLocation2, BezierAlgebraicCuspSemicircleMappedOverlap2,
    BezierAlgebraicCuspSemicirclePairIntersections2, BezierAlgebraicCuspSemicirclePairOverlap2,
    BezierAlgebraicCuspSemicircleParallelIntersections2, BezierAlgebraicCuspSemicircleParameter2,
    BezierAlgebraicCuspSemicircleRationalIntersections2, BezierParameterComponentOverlap2,
};
use crate::bezier_split::{BezierSelectedFiberFragment2, BezierSelectedFiberSource2};
use crate::bezier_tangent_order::algebraic_endpoint_tangent_cross_sign;
use crate::classify::{compare_reals, real_sign};
use crate::curve_intersection::{CurveIntersectionBatchCache, CurveIntersectionContext};
use crate::policy::resolve_certified_operation;
use crate::rational_bezier_general::{
    RationalBezierOverlapParameterCorrespondence2, RationalParameterImageMap2,
    exact_contact_point_evidence,
};
use crate::{
    Aabb2, ArcArcIntersection, Axis2, BezierArrangementFragment2, BezierArrangementGraph2,
    BezierEndpoint, BezierEndpointTangentImage2, BezierLineContactRelation,
    BezierLineCrossingDirection, BezierLineImageFitRelation, BezierParallel2,
    BezierParallelPairIntersectionSet2, BezierParameter2, BezierParameterRange2,
    BezierSplitFragment2, BezierSubcurve2, BooleanOp, Classification, ContourPointLocation, Curve2,
    CurveContext, CurveDerivative2, CurveError, CurveFamily2, CurveIntersectionContact2,
    CurveIntersectionOverlap2, CurveIntersectionPairBlocker2, CurveOperation2, CurveOutcome,
    CurveRegion2, CurveRegionLoopRole, CurveRegionParameter2, CurveRegionParameterRange2,
    CurveResult, ExactCurveError, ExactCurveResult, FillRule, LineSeg2, LineSide, QuadraticBezier2,
    RationalBezier2, RationalBezierIntersectionContacts2, RationalBezierIntersectionOverlap2,
    RationalBezierIntersectionPointEvidence2, RationalBezierOverlapOrientation2,
    RationalBezierPointIncidence2, Real, RealSign, RegionPointLocation, Segment2,
    UncertaintyReason,
};

/// Region operand that owns one retained Boolean carrier.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveRegionBooleanOperand2 {
    /// Carrier originates in the first region.
    First,
    /// Carrier originates in the second region.
    Second,
}

/// Stable identity for one retained region-boundary carrier.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionCarrierRef2 {
    carrier_index: usize,
    operand: CurveRegionBooleanOperand2,
    loop_index: usize,
    fragment_index: usize,
    family: CurveFamily2,
}

/// One exact contact between retained carriers from two curved regions.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionIntersectionContact2 {
    first: CurveRegionCarrierRef2,
    second: CurveRegionCarrierRef2,
    evidence: RegionPairContactEvidence,
}

/// One certified positive-length shared span between two curved regions.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionIntersectionOverlap2 {
    first: CurveRegionCarrierRef2,
    second: CurveRegionCarrierRef2,
    source: Option<CurveIntersectionOverlap2>,
    first_range: CurveRegionParameterRange2,
    second_range: CurveRegionParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
}

/// One incomplete retained carrier pair in a curved-region intersection result.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionIntersectionBlocker2 {
    first: CurveRegionCarrierRef2,
    second: CurveRegionCarrierRef2,
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
struct CurveRegionBooleanContext<'a> {
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
    bezier_self_intersections: Vec<BezierSelfIntersectionCache>,
    parallel_self_intersections: Vec<ParallelSelfIntersectionCache>,
    strict_line_image_only: OnceLock<bool>,
}

#[derive(Clone, Copy)]
enum RetainedPointProbeClassification {
    FilledRegion,
    LoopParity,
}

#[derive(Debug)]
struct ParallelSelfIntersectionCache {
    parallel: BezierParallel2,
    result: OnceLock<CurveResult<Classification<BezierParallelPairIntersectionSet2>>>,
}

#[derive(Debug)]
struct BezierSelfIntersectionCache {
    curve: BezierSubcurve2,
    result: OnceLock<CurveResult<Classification<RationalBezierIntersectionContacts2>>>,
}

#[derive(Clone, Debug)]
struct RegionCarrier {
    operand: CurveRegionBooleanOperand2,
    loop_index: usize,
    fragment_index: usize,
    family: CurveFamily2,
    geometry: RegionCarrierGeometry,
    start: CurveRegionParameter2,
    end: CurveRegionParameter2,
    reversed: bool,
    filled_side_is_left: bool,
    selected_fiber_endpoint_points: Option<Arc<[RationalBezierIntersectionPointEvidence2; 2]>>,
    image_is_injective: OnceLock<bool>,
    bounds: OnceLock<Classification<Aabb2>>,
}

#[derive(Debug)]
struct RegionCarrierPair {
    first_carrier_index: usize,
    second_carrier_index: usize,
    context: RegionCarrierPairContext,
}

#[derive(Clone, Debug)]
enum RegionCarrierGeometry {
    Bezier(BezierSubcurve2),
    AnalyticParallel(BezierParallel2),
    AlgebraicChord(crate::BezierAlgebraicChord2),
    AlgebraicCuspSemicircle(crate::BezierAlgebraicCuspSemicircleFragment2),
}

#[derive(Debug)]
enum RegionCarrierPairContext {
    Bezier(CurveIntersectionContext),
    ParallelRational { parallel_is_first: bool },
    ParallelPair,
    ParallelSameImage,
    ParallelSelf,
    BezierSelf,
    AlgebraicChordPair,
    CuspChord { cusp_is_first: bool },
    CuspRational { cusp_is_first: bool },
    CuspParallel { cusp_is_first: bool },
    CuspPair,
}

#[derive(Clone, Debug, PartialEq)]
struct RegionPairContactEvidence {
    first_parameter: CurveRegionParameter2,
    second_parameter: CurveRegionParameter2,
    point: Option<RationalBezierIntersectionPointEvidence2>,
    certified_transverse: bool,
    tangent_cross_sign: Option<RealSign>,
    tangent_dot_sign: Option<RealSign>,
    second_side_of_first: Option<LineSide>,
}

#[derive(Clone, Debug)]
struct RegionPairOverlap {
    source: Option<RegionPairOverlapSource>,
    first_range: CurveRegionParameterRange2,
    second_range: CurveRegionParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
}

#[derive(Clone, Debug)]
enum RegionPairOverlapSource {
    Bezier(CurveIntersectionOverlap2),
    ParameterComponent {
        source: BezierParameterComponentOverlap2,
        swapped: bool,
    },
    AlgebraicChordRational(BezierAlgebraicChordRationalOverlap2),
    AlgebraicCusp(BezierAlgebraicCuspSemicirclePairOverlap2),
    AlgebraicCuspMapped(BezierAlgebraicCuspSemicircleMappedOverlap2),
    AlgebraicCuspSelectedFiberMapped(BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2),
}

fn parameter_component_region_overlap_sources(
    sources: &[BezierParameterComponentOverlap2],
    overlap: &RationalBezierIntersectionOverlap2,
    swapped: bool,
) -> Vec<Option<RegionPairOverlapSource>> {
    let mut retained = sources
        .iter()
        .filter(|source| source.overlap() == overlap)
        .cloned()
        .map(|source| Some(RegionPairOverlapSource::ParameterComponent { source, swapped }))
        .collect::<Vec<_>>();
    if retained.is_empty() {
        retained.push(None);
    }
    retained
}

#[derive(Clone, Copy)]
enum RegionCuspMappedOverlapRef<'a> {
    Bezier(&'a BezierAlgebraicCuspSemicircleMappedOverlap2),
    Selected(&'a BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2),
}

impl RegionCuspMappedOverlapRef<'_> {
    fn clipped_curve_region_ranges(
        self,
        cusp_fragment: &CurveRegionParameterRange2,
        other_fragment: &CurveRegionParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(CurveRegionParameterRange2, CurveRegionParameterRange2)>>>
    {
        match self {
            Self::Bezier(source) => {
                source.clipped_curve_region_ranges(cusp_fragment, other_fragment, policy)
            }
            Self::Selected(source) => {
                source.clipped_curve_region_ranges(cusp_fragment, other_fragment, policy)
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
enum RegionPairBlocker {
    Bezier(CurveIntersectionPairBlocker2),
    Uncertain(UncertaintyReason),
    IncompleteReplay,
    PointImageParameterComponent,
}

#[derive(Clone, Debug)]
struct RegionPairResult {
    contacts: Vec<RegionPairContactEvidence>,
    overlaps: Vec<RegionPairOverlap>,
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
    parameter: CurveRegionParameter2,
    topology_vertex: Option<usize>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CarrierParameterLocation {
    Outside,
    Endpoint,
    Interior,
}

#[derive(Clone, Debug)]
struct ContactVertex {
    point: Option<RationalBezierIntersectionPointEvidence2>,
    topology_vertex: usize,
    carrier_indices: [usize; 2],
    parameters: [CurveRegionParameter2; 2],
}

#[derive(Clone, Debug)]
struct CarrierOverlap {
    first_carrier_index: usize,
    second_carrier_index: usize,
    first_range: CurveRegionParameterRange2,
    second_range: CurveRegionParameterRange2,
    first_endpoint_vertices: [usize; 2],
    second_endpoint_vertices: [usize; 2],
    orientation: RationalBezierOverlapOrientation2,
}

impl CarrierOverlap {
    fn endpoint_vertices(&self, carrier_index: usize) -> Option<[usize; 2]> {
        if carrier_index == self.first_carrier_index {
            Some(self.first_endpoint_vertices)
        } else if carrier_index == self.second_carrier_index {
            Some(self.second_endpoint_vertices)
        } else {
            None
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
    carrier_index: usize,
    splits: &[SplitCarrierFragment],
) -> Option<std::ops::Range<usize>> {
    let [first_vertex, second_vertex] = overlap.endpoint_vertices(carrier_index)?;
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

#[derive(Debug)]
enum CarrierOverlapClip {
    Unmatched,
    Matched(Option<(CurveRegionParameterRange2, CurveRegionParameterRange2)>),
}

#[derive(Clone, Debug)]
struct TransitionContactCandidate {
    first_carrier: usize,
    second_carrier: usize,
    /// Both contact parameters lie strictly inside their individual carrier
    /// domains. Only these contacts can seed per-carrier Boolean locations;
    /// endpoint contacts instead borrow the neighboring authored branches in
    /// the regularization face kernel.
    interior_on_both_carriers: bool,
    certified_transverse: bool,
    cross_is_positive: Option<bool>,
    tangent_dot_is_positive: Option<bool>,
    second_side_of_first: Option<LineSide>,
    self_parameters: Option<[CurveRegionParameter2; 2]>,
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

#[derive(Debug)]
struct RegularizedFaceUnion {
    parent: Vec<usize>,
}

impl RegularizedFaceUnion {
    fn new(count: usize) -> Self {
        Self {
            parent: (0..count).collect(),
        }
    }

    fn find(&mut self, mut node: usize) -> usize {
        let mut root = node;
        while self.parent[root] != root {
            root = self.parent[root];
        }
        while self.parent[node] != node {
            let parent = self.parent[node];
            self.parent[node] = root;
            node = parent;
        }
        root
    }

    fn unite(&mut self, first: usize, second: usize) {
        let first = self.find(first);
        let second = self.find(second);
        if first == second {
            return;
        }
        let (root, child) = if first < second {
            (first, second)
        } else {
            (second, first)
        };
        self.parent[child] = root;
    }
}

/// Unites an exact local face sector and retains that direct (non-transitive)
/// adjacency for boundary traversal. Global face-component identity is not
/// sufficient at a crossing because one connected face can occupy multiple
/// local sectors around the same vertex.
fn unite_regularized_vertex_sectors(
    face_union: &mut RegularizedFaceUnion,
    vertex_sector_links: &mut Vec<(usize, usize, usize)>,
    vertex: usize,
    pairs: &[(usize, usize)],
) {
    for &(first, second) in pairs {
        face_union.unite(first, second);
        vertex_sector_links.push((vertex, first, second));
    }
}

#[derive(Clone, Copy, Debug)]
struct RegularizedFaceAdjacency {
    face: usize,
    jump: usize,
    direction: i8,
}

#[derive(Clone, Debug)]
enum RegularizedWindingJump {
    Single(usize),
    Aggregate(Box<[(usize, i32)]>),
}

fn propagate_regularized_face_windings(
    face_windings: &mut [Option<Vec<i32>>],
    adjacency: &[Vec<RegularizedFaceAdjacency>],
    jumps: &[RegularizedWindingJump],
    seeds: impl IntoIterator<Item = (usize, Vec<i32>)>,
) -> Result<(), CurveError> {
    let mut queue = std::collections::VecDeque::new();
    for (face, winding) in seeds {
        match face_windings.get(face) {
            Some(Some(existing)) if existing != &winding => {
                return Err(CurveError::Topology(
                    "regularized arrangement assigned inconsistent winding vectors to one face"
                        .into(),
                ));
            }
            Some(Some(_)) => {}
            Some(None) => {
                face_windings[face] = Some(winding);
                queue.push_back(face);
            }
            None => {
                return Err(CurveError::Topology(
                    "regularized arrangement referenced a missing face".into(),
                ));
            }
        }
    }
    while let Some(face) = queue.pop_front() {
        let source = face_windings[face]
            .as_ref()
            .expect("queued regularized face has a winding vector")
            .clone();
        for edge in &adjacency[face] {
            let mut target = source.clone();
            let jump = jumps.get(edge.jump).ok_or_else(|| {
                CurveError::Topology("regularized arrangement references a missing jump".into())
            })?;
            let mut apply = |loop_index: usize, delta: i32| -> Result<(), CurveError> {
                let Some(component) = target.get_mut(loop_index) else {
                    return Err(CurveError::Topology(
                        "regularized arrangement edge references a missing loop".into(),
                    ));
                };
                *component = component
                    .checked_add(delta.saturating_mul(i32::from(edge.direction)))
                    .ok_or_else(|| {
                        CurveError::Topology(
                            "regularized arrangement winding overflowed i32".into(),
                        )
                    })?;
                Ok(())
            };
            match jump {
                RegularizedWindingJump::Single(loop_index) => apply(*loop_index, 1)?,
                RegularizedWindingJump::Aggregate(components) => {
                    for &(loop_index, delta) in components.as_ref() {
                        apply(loop_index, delta)?;
                    }
                }
            }
            match &face_windings[edge.face] {
                Some(existing) if existing != &target => {
                    return Err(CurveError::Topology(
                        "regularized arrangement has inconsistent winding equations".into(),
                    ));
                }
                Some(_) => {}
                None => {
                    face_windings[edge.face] = Some(target);
                    queue.push_back(edge.face);
                }
            }
        }
    }
    Ok(())
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
            let bounds = match carrier
                .geometry
                .certified_outer_bounds_refined(refinement_steps, policy)
            {
                Classification::Decided(bounds) => bounds,
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                    complete = false;
                    break;
                }
            };
            let bounds = bounds.certified_rational_outer_envelope().unwrap_or(bounds);
            accumulated = Some(match accumulated {
                None => bounds,
                Some(ref accumulated) => match accumulated.union(&bounds, &CurveContext::STRICT) {
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

impl CurveRegionCarrierRef2 {
    /// Returns the flattened carrier index in the retained pair.
    pub const fn carrier_index(&self) -> usize {
        self.carrier_index
    }

    /// Returns the region operand that owns this carrier.
    pub const fn operand(&self) -> CurveRegionBooleanOperand2 {
        self.operand
    }

    /// Returns the retained boundary-loop index in its operand.
    pub const fn loop_index(&self) -> usize {
        self.loop_index
    }

    /// Returns the retained fragment index in its boundary loop.
    pub const fn fragment_index(&self) -> usize {
        self.fragment_index
    }

    /// Returns the exact carrier family used by intersection dispatch.
    pub const fn family(&self) -> CurveFamily2 {
        self.family
    }
}

impl CurveRegionIntersectionContact2 {
    /// Returns the first-region carrier identity.
    pub const fn first(&self) -> &CurveRegionCarrierRef2 {
        &self.first
    }

    /// Returns the second-region carrier identity.
    pub const fn second(&self) -> &CurveRegionCarrierRef2 {
        &self.second
    }

    /// Returns the exact parameter on the first retained carrier.
    pub const fn first_parameter(&self) -> &CurveRegionParameter2 {
        self.evidence.first_parameter()
    }

    /// Returns the exact parameter on the second retained carrier.
    pub const fn second_parameter(&self) -> &CurveRegionParameter2 {
        self.evidence.second_parameter()
    }

    /// Returns retained affine point evidence when the pair kernel constructs it.
    ///
    /// Analytic-parallel pairs deliberately retain the two exact parameters as
    /// the point construction and therefore return `None` without demoting the
    /// contact to rounded coordinates.
    pub const fn point(&self) -> Option<&RationalBezierIntersectionPointEvidence2> {
        self.evidence.point()
    }

    /// Returns whether exact tangent evidence certifies a transverse crossing.
    pub const fn is_certified_transverse(&self) -> bool {
        self.evidence.is_certified_transverse()
    }
}

impl CurveRegionIntersectionOverlap2 {
    /// Returns the first-region carrier identity.
    pub const fn first(&self) -> &CurveRegionCarrierRef2 {
        &self.first
    }

    /// Returns the second-region carrier identity.
    pub const fn second(&self) -> &CurveRegionCarrierRef2 {
        &self.second
    }

    /// Returns native top-level overlap evidence when that pair kernel supplied it.
    ///
    /// Analytic pair kernels retain the exact clipped ranges and orientation
    /// directly, so they do not allocate a second native overlap wrapper.
    pub const fn source(&self) -> Option<&CurveIntersectionOverlap2> {
        self.source.as_ref()
    }

    /// Returns the exact overlap range clipped to the first retained carrier.
    pub const fn first_range(&self) -> &CurveRegionParameterRange2 {
        &self.first_range
    }

    /// Returns the exact overlap range clipped to the second retained carrier.
    pub const fn second_range(&self) -> &CurveRegionParameterRange2 {
        &self.second_range
    }

    /// Returns relative source-curve traversal orientation.
    pub const fn orientation(&self) -> RationalBezierOverlapOrientation2 {
        self.orientation
    }
}

impl CurveRegionIntersectionBlocker2 {
    /// Returns the first-region carrier identity.
    pub const fn first(&self) -> &CurveRegionCarrierRef2 {
        &self.first
    }

    /// Returns the second-region carrier identity.
    pub const fn second(&self) -> &CurveRegionCarrierRef2 {
        &self.second
    }

    /// Returns native top-level blocker evidence, when applicable.
    pub const fn native_blocker(&self) -> Option<&CurveIntersectionPairBlocker2> {
        match &self.blocker {
            RegionPairBlocker::Bezier(blocker) => Some(blocker),
            RegionPairBlocker::Uncertain(_)
            | RegionPairBlocker::IncompleteReplay
            | RegionPairBlocker::PointImageParameterComponent => None,
        }
    }

    /// Returns the terminal uncertainty reason when the exact carrier kernel was undecided.
    pub const fn uncertainty_reason(&self) -> Option<UncertaintyReason> {
        match self.blocker {
            RegionPairBlocker::Uncertain(reason) => Some(reason),
            RegionPairBlocker::Bezier(_)
            | RegionPairBlocker::IncompleteReplay
            | RegionPairBlocker::PointImageParameterComponent => None,
        }
    }

    /// Returns true when exact replay retained candidates it could not complete.
    pub const fn is_incomplete_replay(&self) -> bool {
        matches!(self.blocker, RegionPairBlocker::IncompleteReplay)
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
    fn from_bezier(contact: &CurveIntersectionContact2) -> Self {
        Self {
            first_parameter: CurveRegionParameter2::from_bezier(
                contact.first().local_parameter().clone(),
            ),
            second_parameter: CurveRegionParameter2::from_bezier(
                contact.second().local_parameter().clone(),
            ),
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
        point: Option<RationalBezierIntersectionPointEvidence2>,
        certified_transverse: bool,
        tangent_cross_sign: Option<RealSign>,
    ) -> Self {
        Self::direct(
            CurveRegionParameter2::from_bezier(first_parameter),
            CurveRegionParameter2::from_bezier(second_parameter),
            point,
            certified_transverse,
            tangent_cross_sign,
        )
    }

    fn direct(
        first_parameter: CurveRegionParameter2,
        second_parameter: CurveRegionParameter2,
        point: Option<RationalBezierIntersectionPointEvidence2>,
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

    const fn first_parameter(&self) -> &CurveRegionParameter2 {
        &self.first_parameter
    }

    const fn second_parameter(&self) -> &CurveRegionParameter2 {
        &self.second_parameter
    }

    const fn point(&self) -> Option<&RationalBezierIntersectionPointEvidence2> {
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
        if policy.permits_approximate_512() {
            match policy
                .strict_predicate_pass(|| self.boolean_region_once(other, operation, policy))
            {
                Ok(region) => return Ok(region),
                Err(ExactCurveError::Blocked(_)) => {}
                Err(error) => return Err(error),
            }
        }
        self.boolean_region_once(other, operation, policy)
    }

    fn boolean_region_once(
        &self,
        other: &Self,
        operation: BooleanOp,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if let Some(region) = boolean_trivial_region(self, other, operation)? {
            return Ok(region);
        }
        CurveRegionBooleanContext::try_new(self, other, policy)?
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
        if policy.permits_approximate_512() {
            match policy.strict_predicate_pass(|| self.boolean_regions_once(other, policy)) {
                Ok(regions) => return Ok(regions),
                Err(ExactCurveError::Blocked(_)) => {}
                Err(error) => return Err(error),
            }
        }
        self.boolean_regions_once(other, policy)
    }

    fn boolean_regions_once(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveRegionBooleanResults2> {
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
        if self.is_empty() || other.is_empty() || self == other {
            let immediate = [
                boolean_trivial_region(self, other, operations[0])?,
                boolean_trivial_region(self, other, operations[1])?,
                boolean_trivial_region(self, other, operations[2])?,
                boolean_trivial_region(self, other, operations[3])?,
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
        CurveRegionBooleanContext::try_new(self, other, policy)?.build_boolean_regions()
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
        if policy.permits_approximate_512() {
            match policy.strict_predicate_pass(|| self.regularized_region_once(policy)) {
                Ok(region) => return Ok(region),
                Err(ExactCurveError::Blocked(_)) => {}
                Err(error) => return Err(error),
            }
        }
        self.regularized_region_once(policy)
    }

    fn regularized_region_once(&self, policy: &CurveContext) -> ExactCurveResult<Self> {
        if self.is_empty() {
            return Ok(self.clone());
        }
        // An authoritative filled-left face walk or an independent exact
        // convex-boundary certificate is already a canonical regularization
        // proof. Rebuilding its arrangement wastes work and, for compact
        // correlated chord cuts, would throw away the topology evidence that
        // deliberately replaces coordinate materialization.
        if self.has_certified_regularized_filled_left_topology() {
            return Ok(self.clone());
        }
        CurveRegionBooleanContext::try_new_unary(self, policy)
            .and_then(|context| context.build_regularized_region())
            .map_err(|error| error.with_operation(CurveOperation2::Arrangement))
    }

    /// Collects exact contacts and overlaps against another region immediately.
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
        CurveRegionBooleanContext::try_new(self, other, policy)?.build_intersection_evidence()
    }

    pub(crate) fn intersect_curve_boundary_carriers_raw(
        &self,
        curve: &Curve2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveRegionIntersectionResult2> {
        CurveRegionBooleanContext::try_new_curve_boundary(curve, self, policy)
            .and_then(|context| context.build_intersection_evidence())
            .map_err(|error| error.with_operation(CurveOperation2::Subdivision))
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
    point: RationalBezierIntersectionPointEvidence2,
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
                bezier_self_intersections: Vec::new(),
                parallel_self_intersections: Vec::new(),
                strict_line_image_only: OnceLock::new(),
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
        let curves = carriers
            .iter()
            .map(|carrier| match &carrier.geometry {
                RegionCarrierGeometry::Bezier(curve) => Some(Curve2::from(curve.clone())),
                RegionCarrierGeometry::AnalyticParallel(_)
                | RegionCarrierGeometry::AlgebraicChord(_)
                | RegionCarrierGeometry::AlgebraicCuspSemicircle(_) => None,
            })
            .collect::<Vec<_>>();
        let mut pairs = Vec::with_capacity(
            first_carrier_count
                .saturating_add(carriers.len() - first_carrier_count)
                .min(authored_carrier_pair_count),
        );
        let mut intersection_cache = CurveIntersectionBatchCache::default();
        for first_carrier_index in 0..first_carrier_count {
            for second_carrier_index in first_carrier_count..carriers.len() {
                if let Some(pair) = build_candidate_carrier_pair(
                    &carriers,
                    &curves,
                    first_carrier_index,
                    second_carrier_index,
                    policy,
                    &mut intersection_cache,
                )? {
                    pairs.push(pair);
                }
            }
        }
        let bezier_self_intersections = build_bezier_self_intersection_caches(&carriers, &pairs);
        let parallel_self_intersections = build_parallel_self_intersection_caches(&carriers);

        Ok(Self {
            data: CurveRegionBooleanContextData {
                first,
                second,
                policy: *policy,
                carriers,
                first_carrier_count,
                authored_carrier_pair_count,
                pairs,
                bezier_self_intersections,
                parallel_self_intersections,
                strict_line_image_only: OnceLock::new(),
            },
        })
    }

    fn try_new_curve_boundary(
        source: &Curve2,
        region: &'a CurveRegion2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let source_fragments =
            source.native_bezier_fragments_for_operation(policy, CurveOperation2::Subdivision)?;
        let mut carriers = Vec::with_capacity(
            source_fragments
                .len()
                .saturating_add(region_carrier_count(region)),
        );
        for (fragment_index, fragment) in source_fragments.iter().enumerate() {
            let geometry = RegionCarrierGeometry::Bezier(fragment.curve().clone());
            carriers.push(RegionCarrier {
                operand: CurveRegionBooleanOperand2::First,
                loop_index: 0,
                fragment_index,
                family: geometry.family(),
                geometry,
                start: CurveRegionParameter2::from_bezier(BezierParameter2::Exact(Real::zero())),
                end: CurveRegionParameter2::from_bezier(BezierParameter2::Exact(Real::one())),
                reversed: false,
                filled_side_is_left: false,
                selected_fiber_endpoint_points: None,
                image_is_injective: OnceLock::new(),
                bounds: OnceLock::new(),
            });
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
        let second_carrier_count = carriers.len() - first_carrier_count;
        let curves = carriers
            .iter()
            .map(|carrier| match &carrier.geometry {
                RegionCarrierGeometry::Bezier(curve) => Some(Curve2::from(curve.clone())),
                RegionCarrierGeometry::AnalyticParallel(_)
                | RegionCarrierGeometry::AlgebraicChord(_)
                | RegionCarrierGeometry::AlgebraicCuspSemicircle(_) => None,
            })
            .collect::<Vec<_>>();
        let mut pairs = Vec::with_capacity(
            first_carrier_count
                .saturating_add(second_carrier_count)
                .min(authored_carrier_pair_count),
        );
        let mut intersection_cache = CurveIntersectionBatchCache::default();
        for first_carrier_index in 0..first_carrier_count {
            for second_carrier_index in first_carrier_count..carriers.len() {
                if let Some(pair) = build_candidate_carrier_pair(
                    &carriers,
                    &curves,
                    first_carrier_index,
                    second_carrier_index,
                    policy,
                    &mut intersection_cache,
                )? {
                    pairs.push(pair);
                }
            }
        }

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
                bezier_self_intersections: Vec::new(),
                parallel_self_intersections: Vec::new(),
                strict_line_image_only: OnceLock::new(),
            },
        })
    }

    fn try_new_unary(region: &'a CurveRegion2, policy: &'a CurveContext) -> ExactCurveResult<Self> {
        let mut rational_quadratic_area_cache = RationalQuadraticAreaIntegralCache::default();
        let carriers = build_region_carriers(
            region,
            CurveRegionBooleanOperand2::First,
            policy,
            &mut rational_quadratic_area_cache,
            false,
        )?;
        let carrier_count = carriers.len();
        let authored_carrier_pair_count =
            carrier_count.saturating_mul(carrier_count.saturating_add(1)) / 2;
        let curves = carriers
            .iter()
            .map(|carrier| match &carrier.geometry {
                RegionCarrierGeometry::Bezier(curve) => Some(Curve2::from(curve.clone())),
                RegionCarrierGeometry::AnalyticParallel(_)
                | RegionCarrierGeometry::AlgebraicChord(_)
                | RegionCarrierGeometry::AlgebraicCuspSemicircle(_) => None,
            })
            .collect::<Vec<_>>();
        let mut pairs = Vec::with_capacity(carrier_count.saturating_mul(2));
        let mut intersection_cache = CurveIntersectionBatchCache::default();
        for first_carrier_index in 0..carrier_count {
            for second_carrier_index in first_carrier_index + 1..carrier_count {
                let candidate = build_candidate_carrier_pair(
                    &carriers,
                    &curves,
                    first_carrier_index,
                    second_carrier_index,
                    policy,
                    &mut intersection_cache,
                );
                if let Some(pair) = candidate? {
                    pairs.push(pair);
                }
            }
            let carrier = &carriers[first_carrier_index];
            if !carrier.geometry.has_certified_injective_image(policy) {
                pairs.push(RegionCarrierPair {
                    first_carrier_index,
                    second_carrier_index: first_carrier_index,
                    context: match &carrier.geometry {
                        RegionCarrierGeometry::AnalyticParallel(_) => {
                            RegionCarrierPairContext::ParallelSelf
                        }
                        RegionCarrierGeometry::Bezier(_) => RegionCarrierPairContext::BezierSelf,
                        RegionCarrierGeometry::AlgebraicChord(_) => {
                            unreachable!("an algebraic chord is an injective retained carrier")
                        }
                        RegionCarrierGeometry::AlgebraicCuspSemicircle(_) => unreachable!(
                            "an algebraic cusp semicircle is an injective retained carrier"
                        ),
                    },
                });
            }
        }
        let bezier_self_intersections = build_bezier_self_intersection_caches(&carriers, &pairs);
        let parallel_self_intersections = build_parallel_self_intersection_caches(&carriers);

        Ok(Self {
            data: CurveRegionBooleanContextData {
                first: region,
                second: region,
                policy: *policy,
                carriers,
                first_carrier_count: carrier_count,
                authored_carrier_pair_count,
                pairs,
                bezier_self_intersections,
                parallel_self_intersections,
                strict_line_image_only: OnceLock::new(),
            },
        })
    }

    fn intersect_algebraic_probe_boundary(
        &self,
        probe: crate::BezierAlgebraicChord2,
    ) -> ExactCurveResult<CurveRegionIntersectionResult2> {
        self.intersect_algebraic_probe_carriers(probe, self.data.first, &self.data.carriers)
    }

    fn intersect_algebraic_probe_carriers(
        &self,
        probe: crate::BezierAlgebraicChord2,
        boundary_region: &CurveRegion2,
        boundary_carriers: &[RegionCarrier],
    ) -> ExactCurveResult<CurveRegionIntersectionResult2> {
        let start = probe.start_parameter();
        let end = probe.end_parameter();
        let mut carriers = Vec::with_capacity(boundary_carriers.len().saturating_add(1));
        carriers.push(RegionCarrier {
            operand: CurveRegionBooleanOperand2::First,
            loop_index: 0,
            fragment_index: 0,
            family: CurveFamily2::Line,
            geometry: RegionCarrierGeometry::AlgebraicChord(probe),
            start: CurveRegionParameter2::from_algebraic_chord(start),
            end: CurveRegionParameter2::from_algebraic_chord(end),
            reversed: false,
            filled_side_is_left: false,
            selected_fiber_endpoint_points: None,
            image_is_injective: OnceLock::new(),
            bounds: OnceLock::new(),
        });
        carriers.extend(boundary_carriers.iter().cloned().map(|mut carrier| {
            carrier.operand = CurveRegionBooleanOperand2::Second;
            carrier
        }));

        let curves = carriers
            .iter()
            .map(|carrier| match &carrier.geometry {
                RegionCarrierGeometry::Bezier(curve) => Some(Curve2::from(curve.clone())),
                RegionCarrierGeometry::AnalyticParallel(_)
                | RegionCarrierGeometry::AlgebraicChord(_)
                | RegionCarrierGeometry::AlgebraicCuspSemicircle(_) => None,
            })
            .collect::<Vec<_>>();
        let mut pairs = Vec::with_capacity(boundary_carriers.len());
        let mut intersection_cache = CurveIntersectionBatchCache::default();
        for second_carrier_index in 1..carriers.len() {
            if let Some(pair) = build_candidate_carrier_pair(
                &carriers,
                &curves,
                0,
                second_carrier_index,
                &self.data.policy,
                &mut intersection_cache,
            )? {
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
                bezier_self_intersections: Vec::new(),
                parallel_self_intersections: Vec::new(),
                strict_line_image_only: OnceLock::new(),
            },
        }
        .build_intersection_evidence()
    }

    fn build_intersection_evidence(&self) -> ExactCurveResult<CurveRegionIntersectionResult2> {
        let mut contacts = Vec::new();
        let mut overlaps = Vec::new();
        let mut blockers = Vec::new();
        for pair in &self.data.pairs {
            let result = self.pair_result(pair)?;
            let first = self.carrier_ref(pair.first_carrier_index);
            let second = self.carrier_ref(pair.second_carrier_index);
            blockers.extend(result.blockers.into_iter().map(|blocker| {
                CurveRegionIntersectionBlocker2 {
                    first: first.clone(),
                    second: second.clone(),
                    blocker,
                }
            }));
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
                        first: first.clone(),
                        second: second.clone(),
                        evidence: contact,
                    });
                }
            }
            for overlap in result.overlaps {
                let Some((first_range, second_range)) =
                    self.clipped_overlap_ranges(pair, &overlap)?
                else {
                    continue;
                };
                overlaps.push(CurveRegionIntersectionOverlap2 {
                    first: first.clone(),
                    second: second.clone(),
                    source: overlap.source.and_then(|source| match source {
                        RegionPairOverlapSource::Bezier(source) => Some(source),
                        RegionPairOverlapSource::ParameterComponent { .. } => None,
                        RegionPairOverlapSource::AlgebraicChordRational(_) => None,
                        RegionPairOverlapSource::AlgebraicCusp(_)
                        | RegionPairOverlapSource::AlgebraicCuspMapped(_) => None,
                        RegionPairOverlapSource::AlgebraicCuspSelectedFiberMapped(_) => None,
                    }),
                    first_range,
                    second_range,
                    orientation: overlap.orientation,
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

    fn carrier_ref(&self, carrier_index: usize) -> CurveRegionCarrierRef2 {
        let carrier = &self.data.carriers[carrier_index];
        CurveRegionCarrierRef2 {
            carrier_index,
            operand: carrier.operand,
            loop_index: carrier.loop_index,
            fragment_index: carrier.fragment_index,
            family: carrier.family,
        }
    }

    fn parallel_self_intersections(
        &self,
        parallel: &BezierParallel2,
    ) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
        self.data
            .parallel_self_intersections
            .iter()
            .find(|cache| cache.parallel == *parallel)
            .expect("every analytic carrier has an operation-scoped self-intersection cache")
            .result
            .get_or_init(|| parallel.self_intersections(&self.data.policy))
            .clone()
    }

    fn bezier_self_intersections(
        &self,
        curve: &BezierSubcurve2,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        self.data
            .bezier_self_intersections
            .iter()
            .find(|cache| cache.curve == *curve)
            .expect("every Bezier carrier has an operation-scoped self-intersection cache")
            .result
            .get_or_init(|| {
                RationalBezier2::try_from_subcurve(curve)?
                    .self_intersection_contacts_classified(&self.data.policy)
            })
            .clone()
    }

    fn authored_parallel_support_contact(
        &self,
        pair: &RegionCarrierPair,
        parallel: &BezierParallel2,
        parallel_index: usize,
        direction_x: &Real,
        direction_y: &Real,
        regular_range: Option<&CurveRegionParameterRange2>,
    ) -> ExactCurveResult<Option<(Real, RealSign)>> {
        let Some((first_at_start, second_at_start)) = self.authored_carrier_shared_endpoints(pair)
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
        .as_bezier_parameter()
        .and_then(BezierParameter2::as_exact)
        .cloned() else {
            return Ok(None);
        };
        let tangent_relation = match regular_range {
            Some(range) => parallel.vector_tangent_cross_and_dot_signs_on_regular_range(
                &BezierParameter2::Exact(parameter.clone()),
                direction_x,
                direction_y,
                range,
                &self.data.policy,
            ),
            None => parallel.vector_tangent_cross_and_dot_signs(
                &BezierParameter2::Exact(parameter.clone()),
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
        regular_range: Option<&CurveRegionParameterRange2>,
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
            if contact.parameter().as_exact().is_some_and(|parameter| {
                authored_contact
                    .as_ref()
                    .is_some_and(|(authored, _)| parameter == authored)
            }) {
                continue;
            }
            if let Some(certified) = contact.parameter().as_exact().and_then(|parameter| {
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
                    Some(RationalBezierIntersectionPointEvidence2::Exact(point)),
                    false,
                    None,
                ));
        }
        Ok(Classification::Decided(Some(result)))
    }

    fn parallel_arc_pair_result(
        &self,
        parallel: &BezierParallel2,
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
        let parameters = incidence;
        if parameters.iter().all(|(parameter, _)| parameter.is_exact()) {
            match self.parallel_exact_parameter_pair_result(
                parallel,
                curve,
                parameters
                    .iter()
                    .map(|(parameter, _)| parameter.clone())
                    .collect(),
                parallel_is_first,
            )? {
                Classification::Decided(Some(result)) => {
                    return Ok(Classification::Decided(Some(result)));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(_) => {}
            }
        }
        if let BezierSubcurve2::RationalQuadratic(conic) = curve
            && let Some(mut parameter_maps) = parallel
                .circle_rational_quadratic_parameter_maps(
                    arc.center(),
                    arc.radius_squared_ref(),
                    conic,
                )
                .map_err(|cause| self.invalid(0, cause))?
        {
            for (map_index, (numerator, denominator)) in parameter_maps.iter_mut().enumerate() {
                let anchor = if map_index == 0 {
                    conic.start()
                } else {
                    conic.end()
                };
                if let Some(contact) = certified_tangent_contacts
                    .iter()
                    .find(|contact| contact.point == *anchor)
                {
                    // Both homogeneous line coordinates in an inverse conic
                    // chart vanish at its certified anchor contact. Remove
                    // that common source-parameter factor by construction
                    // before elimination instead of replaying nested radicals.
                    if numerator.len() < 2 || denominator.len() < 2 {
                        return Ok(Classification::Decided(None));
                    }
                    *numerator = crate::bezier_parameter::divide_by_linear_root(
                        numerator,
                        &contact.parameter,
                    );
                    *denominator = crate::bezier_parameter::divide_by_linear_root(
                        denominator,
                        &contact.parameter,
                    );
                }
            }
            let mut parameter_maps = parameter_maps
                .into_iter()
                .map(|(numerator, denominator)| {
                    RationalParameterImageMap2::new(numerator, denominator, &self.data.policy)
                })
                .collect::<Vec<_>>();
            let rational = RationalBezier2::try_from_subcurve(curve)
                .map_err(|cause| self.invalid(0, cause))?;
            let mut contacts = Vec::with_capacity(parameters.len());
            for (parallel_parameter, radial_crossing_sign) in &parameters {
                let certified_contact = parallel_parameter.as_exact().and_then(|parameter| {
                    certified_tangent_contacts
                        .iter()
                        .find(|contact| contact.parameter == *parameter)
                });
                if certified_contact.is_none()
                    && let Some(exact) = parallel_parameter.as_exact()
                {
                    let point = parallel
                        .point_at(exact, &self.data.policy)
                        .map_err(|cause| self.invalid(0, cause))?;
                    // This is only a cheap finite-arc rejection. An
                    // undecided represented point must continue through the
                    // exact rational parameter image below.
                    if let Classification::Decided(point) = point
                        && arc.contains_point(&point, &self.data.policy)
                            == Classification::Decided(false)
                    {
                        continue;
                    }
                }
                let other_parameter = if let Some(contact) = certified_contact
                    && contact.point == *conic.start()
                {
                    Some(BezierParameter2::Exact(Real::zero()))
                } else if let Some(contact) = certified_contact
                    && contact.point == *conic.end()
                {
                    Some(BezierParameter2::Exact(Real::one()))
                } else if certified_contact.is_some() {
                    // Join-level tangent certificates are retained by every
                    // minor span. A certified join endpoint that is not this
                    // span's endpoint is outside this span by construction.
                    continue;
                } else {
                    let mut mapped = None;
                    let mut uncertain = None;
                    for parameter_map in &mut parameter_maps {
                        match parameter_map
                            .image(parallel_parameter)
                            .map_err(|cause| self.invalid(0, cause))?
                        {
                            Classification::Decided(Some(parameter)) => {
                                mapped = Some(parameter);
                                break;
                            }
                            Classification::Decided(None) => {}
                            Classification::Uncertain(reason) => uncertain = Some(reason),
                        }
                    }
                    if mapped.is_none()
                        && let Some(reason) = uncertain
                    {
                        return Ok(Classification::Uncertain(reason));
                    }
                    mapped
                };
                let Some(other_parameter) = other_parameter else {
                    continue;
                };
                let point =
                    exact_contact_point_evidence(&rational, &other_parameter, &self.data.policy)
                        .map_err(|cause| self.invalid(0, cause))?;
                let (first_parameter, second_parameter) = if parallel_is_first {
                    (parallel_parameter.clone(), other_parameter)
                } else {
                    (other_parameter, parallel_parameter.clone())
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
                contacts.push(RegionPairContactEvidence::direct_bezier(
                    first_parameter,
                    second_parameter,
                    point,
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
            if let Some(contact) = parameter.as_exact().and_then(|parameter| {
                certified_tangent_contacts
                    .iter()
                    .find(|contact| contact.parameter == *parameter)
            }) {
                if contact.point == *arc.start() || contact.point == *arc.end() {
                    retained_parameters.push(parameter);
                }
                continue;
            }
            let Some(exact) = parameter.as_exact() else {
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
            let Some(parallel_parameter_exact) = parallel_parameter.as_exact() else {
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
                .point_incidence_classified(&point, &self.data.policy)
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
                let Some(other_parameter_exact) = other_parameter.as_exact() else {
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
                let parallel_cross_other = real_sign(&cross, &self.data.policy);
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
                    Some(RationalBezierIntersectionPointEvidence2::Exact(
                        point.clone(),
                    )),
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
        self.authored_carrier_shared_endpoints(pair).is_some()
    }

    /// Returns the authored endpoint shared by each carrier. `true` names its
    /// traversal start and `false` its traversal end.
    fn authored_carrier_shared_endpoints(&self, pair: &RegionCarrierPair) -> Option<(bool, bool)> {
        let first = &self.data.carriers[pair.first_carrier_index];
        let second = &self.data.carriers[pair.second_carrier_index];
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
            .parameter_at_certified_point(
                RationalBezierIntersectionPointEvidence2::Exact(point.clone()),
                &self.data.policy,
            )
            .map_err(|cause| self.invalid(chord_index, cause))?
        {
            Classification::Decided(Some(parameter)) => Ok(Classification::Decided(
                CurveRegionParameter2::from_algebraic_chord(parameter),
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
                    let curve_parameter =
                        CurveRegionParameter2::from_bezier(BezierParameter2::Exact(b_param));
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
                            Some(RationalBezierIntersectionPointEvidence2::Exact(point)),
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
                let chord_range = CurveRegionParameterRange2::new_validated(chord_start, chord_end);
                let (curve_start, curve_end, orientation) =
                    match compare_reals(b_range.start(), b_range.end(), &self.data.policy) {
                        Some(Ordering::Less) => (
                            b_range.start().clone(),
                            b_range.end().clone(),
                            RationalBezierOverlapOrientation2::Same,
                        ),
                        Some(Ordering::Greater) => (
                            b_range.end().clone(),
                            b_range.start().clone(),
                            RationalBezierOverlapOrientation2::Reversed,
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
                let curve_range = CurveRegionParameterRange2::new_validated(
                    CurveRegionParameter2::from_bezier(BezierParameter2::Exact(curve_start)),
                    CurveRegionParameter2::from_bezier(BezierParameter2::Exact(curve_end)),
                );
                let (first_range, second_range) = if chord_is_first {
                    (chord_range, curve_range)
                } else {
                    (curve_range, chord_range)
                };
                RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: vec![RegionPairOverlap {
                        first_range,
                        second_range,
                        orientation,
                        source: None,
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
            let RegionCarrierGeometry::Bezier(source_curve) = &source_carrier.geometry else {
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
            let target_region_parameter =
                CurveRegionParameter2::from_bezier(target_parameter.clone());
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
            let parameter = CurveRegionParameter2::from_bezier(contact.parameter().clone());
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
            let target_parameter = CurveRegionParameter2::from_bezier(contact.parameter().clone());
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
        shared_source_parameter: Option<&BezierParameter2>,
    ) -> ExactCurveResult<Option<RegionPairResult>> {
        let other_index = if chord_index == pair.first_carrier_index {
            pair.second_carrier_index
        } else {
            pair.first_carrier_index
        };
        let source_parameter = match shared_source_parameter {
            Some(BezierParameter2::Algebraic(parameter)) => Some(parameter),
            Some(BezierParameter2::Exact(_)) | None => None,
        };
        let general_intersections = || {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair",
                "general-rational",
            );
            chord
                .rational_intersections(rational, shared_source_parameter, &self.data.policy)
                .map_err(|cause| self.invalid(other_index, cause))
        };
        let intersections = if let Some(source_parameter) = source_parameter {
            match chord
                .source_related_intersections(rational, source_parameter, &self.data.policy)
                .map_err(|cause| self.invalid(other_index, cause))?
            {
                Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::NotSourceRelated
                    | BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                )
                | Classification::Uncertain(UncertaintyReason::Unsupported) => {
                    general_intersections()?
                }
                intersections => intersections,
            }
        } else {
            // The source-related kernel removes its authored endpoint by
            // construction. Only adjacent carriers own that endpoint through
            // common loop topology; every nonadjacent pair requires the
            // complete general contact set.
            general_intersections()?
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
                    CurveRegionParameter2::from_algebraic_chord(contact.chord_parameter().clone());
                let other_parameter =
                    CurveRegionParameter2::from_bezier(contact.other_parameter().clone());
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
                let chord_range = CurveRegionParameterRange2::new_validated(
                    CurveRegionParameter2::from_algebraic_chord(chord_start.clone()),
                    CurveRegionParameter2::from_algebraic_chord(chord_end.clone()),
                );
                let source_range = overlap.source_range().clone();
                let orientation = overlap.orientation();
                let (first_range, second_range) = if chord_is_first {
                    (chord_range, source_range)
                } else {
                    (source_range, chord_range)
                };
                RegionPairOverlap {
                    source: Some(RegionPairOverlapSource::AlgebraicChordRational(overlap)),
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
        if let (Some(start), Some(end)) = (
            parallel_carrier.start.as_bezier_parameter(),
            parallel_carrier.end.as_bezier_parameter(),
        ) {
            let range = BezierParameterRange2::new_validated(start.clone(), end.clone());
            let monotonic = chord
                .parallel_tangent_cross_sign_on_range(parallel, &range, &self.data.policy)
                .map_err(|cause| self.invalid(parallel_index, cause))?;
            if matches!(
                monotonic,
                Classification::Decided(RealSign::Positive | RealSign::Negative)
            ) {
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
                let endpoint_side = |parameter: &BezierParameter2| {
                    let point = RationalBezierIntersectionPointEvidence2::AnalyticParallel(
                        crate::BezierAnalyticParallelPoint2::new(
                            parallel.clone(),
                            parameter.clone(),
                            &self.data.policy,
                        ),
                    );
                    let certified = chord.certified_tangent_side(&point, &self.data.policy);
                    if matches!(certified, Classification::Decided(_)) {
                        Ok(certified)
                    } else {
                        chord.strict_oriented_side_by_fast_refinement(&point, &self.data.policy)
                    }
                };
                let sides = [endpoint_side(start), endpoint_side(end)];
                let sides = match sides {
                    [Ok(first), Ok(second)] => [first, second],
                    [Err(cause), _] | [_, Err(cause)] => {
                        return Err(self.invalid(chord_index, cause));
                    }
                };
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
            }
        }
        // Carrier representation is structural. Probe it under STRICT so
        // APPROXIMATE_512 remains terminal evidence rather than dispatch.
        if !self.authored_carriers_are_adjacent(pair)
            && parallel_carrier.start.as_bezier_parameter().is_some()
            && parallel_carrier.end.as_bezier_parameter().is_some()
            && let Classification::Decided(Some(rational)) = parallel
                .exact_rational_parallel_component(&CurveContext::STRICT)
                .map_err(|cause| self.invalid(parallel_index, cause))?
            && let Some(result) = self.algebraic_chord_rational_pair_result(
                pair,
                chord,
                chord_index,
                &rational,
                None,
            )?
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair",
                "analytic-parallel-strict-rational-component",
            );
            return Ok(result);
        }
        let support_result = self.algebraic_chord_parallel_support_pair_result(
            pair,
            chord,
            chord_index,
            parallel,
            parallel_index,
        )?;
        if support_result.blockers.is_empty() {
            return Ok(support_result);
        }
        {
            let blocker = |reason| RegionPairResult {
                contacts: Vec::new(),
                overlaps: Vec::new(),
                blockers: vec![RegionPairBlocker::Uncertain(reason)],
            };
            let retained = match (
                parallel_carrier.start.as_bezier_parameter(),
                parallel_carrier.end.as_bezier_parameter(),
            ) {
                (Some(start), Some(end)) => chord.parallel_intersections_on_regular_range(
                    parallel,
                    &BezierParameterRange2::new_validated(start.clone(), end.clone()),
                    &self.data.policy,
                ),
                _ => chord.parallel_intersections(parallel, &self.data.policy),
            }
            .map_err(|cause| self.invalid(parallel_index, cause))?;
            let contacts = match retained {
                Classification::Decided(BezierAlgebraicChordParallelIntersections2::Contacts(
                    contacts,
                )) => Some(contacts),
                Classification::Decided(
                    BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                ) if chord.exact_line().is_none() => {
                    return Ok(blocker(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) if chord.exact_line().is_none() => {
                    return Ok(blocker(reason));
                }
                Classification::Decided(
                    BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                )
                | Classification::Uncertain(_) => None,
            };
            if let Some(contacts) = contacts {
                let chord_is_first = chord_index == pair.first_carrier_index;
                let contacts = contacts
                    .into_iter()
                    .map(|contact| {
                        let tangent_cross_sign =
                            orient_tangent_cross_sign(contact.tangent_cross_sign(), chord_is_first);
                        let chord_parameter = CurveRegionParameter2::from_algebraic_chord(
                            contact.chord_parameter().clone(),
                        );
                        let parallel_parameter = CurveRegionParameter2::from_bezier(
                            contact.parallel_parameter().clone(),
                        );
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
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "analytic-parallel-retained-support",
                );
                return Ok(RegionPairResult {
                    contacts,
                    overlaps: Vec::new(),
                    blockers: Vec::new(),
                });
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
        let chord_parameter = |point: RationalBezierIntersectionPointEvidence2| match chord
            .parameter_at_certified_point(point, &self.data.policy)
            .map_err(|cause| self.invalid(chord_index, cause))?
        {
            Classification::Decided(Some(parameter)) => Ok(Classification::Decided(
                CurveRegionParameter2::from_algebraic_chord(parameter),
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
                CurveRegionParameterRange2::new_validated(
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
            let parallel_parameter =
                CurveRegionParameter2::from_bezier(contact.parallel_parameter().clone());
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
                let point =
                    exact_contact_point_evidence(&rational_line, parameter, &self.data.policy)
                        .map_err(|cause| self.invalid(chord_index, cause))?
                        .ok_or_else(|| {
                            self.invalid(
                                chord_index,
                                CurveError::Topology(
                                    "a parallel/line overlap endpoint lost its exact point image"
                                        .into(),
                                ),
                            )
                        })?;
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
            let chord_range = CurveRegionParameterRange2::new_validated(chord_start, chord_end);
            let parallel_range =
                CurveRegionParameterRange2::from_bezier_range(overlap.first_range().clone());
            let (first_range, second_range) = if chord_is_first {
                (chord_range, parallel_range)
            } else {
                (parallel_range, chord_range)
            };
            overlaps.push(RegionPairOverlap {
                first_range,
                second_range,
                orientation: overlap.orientation(),
                source: None,
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
        let regular_range = parallel_carrier
            .start
            .as_bezier_parameter()
            .zip(parallel_carrier.end.as_bezier_parameter())
            .map(|_| {
                CurveRegionParameterRange2::new_validated(
                    parallel_carrier.start.clone(),
                    parallel_carrier.end.clone(),
                )
            });
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
        let Some((direction_x, direction_y)) =
            authored_direction.or_else(|| chord.certified_unit_tangent())
        else {
            return Ok(blocker(UncertaintyReason::Unsupported));
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
            regular_range.as_ref(),
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
        let relation_on_retained_range = |deep_branch_refinement| match regular_range.as_ref() {
            Some(range) => parallel
                .relation_to_supporting_line_on_regular_range_with_certified_contacts(
                    &directed_line,
                    range,
                    authored_crossing,
                    &certified_tangent_parameters,
                    deep_branch_refinement,
                    &self.data.policy,
                ),
            None => parallel.relation_to_supporting_line_with_direction_and_certified_contacts(
                &directed_line,
                &direction_x,
                &direction_y,
                authored_crossing,
                &certified_tangent_parameters,
                deep_branch_refinement,
                &self.data.policy,
            ),
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
                    .supporting_line_squared_incidence(&directed_line, &self.data.policy)
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
                            crate::RationalBezierIntersectionPointEvidence2::AnalyticParallel(
                                crate::BezierAnalyticParallelPoint2::new(
                                    parallel.clone(),
                                    parameter.clone(),
                                    &self.data.policy,
                                ),
                            );
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
            if contact.parameter().as_exact().is_some_and(|parameter| {
                authored_contact
                    .as_ref()
                    .is_some_and(|(authored, _)| parameter == authored)
            }) {
                continue;
            }
            let certified = contact.parameter().as_exact().and_then(|parameter| {
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
                let point = if let Some(range) = regular_range.as_ref() {
                    match parallel
                        .point_evidence_on_regular_range(
                            contact.parameter(),
                            range,
                            &self.data.policy,
                        )
                        .map_err(|cause| self.invalid(parallel_index, cause))?
                    {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => return Ok(blocker(reason)),
                    }
                } else if contact.parameter().as_exact().is_some() {
                    match parallel
                        .supporting_line_contact_evidence(
                            &directed_line,
                            contact.parameter(),
                            &self.data.policy,
                        )
                        .map_err(|cause| self.invalid(parallel_index, cause))?
                    {
                        Classification::Decided((point, _)) => point,
                        Classification::Uncertain(reason) => return Ok(blocker(reason)),
                    }
                } else {
                    crate::RationalBezierIntersectionPointEvidence2::AnalyticParallel(
                        crate::BezierAnalyticParallelPoint2::new(
                            parallel.clone(),
                            contact.parameter().clone(),
                            &self.data.policy,
                        ),
                    )
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
                let tangent_relation = match regular_range.as_ref() {
                    Some(range) => parallel.vector_tangent_cross_and_dot_signs_on_regular_range(
                        contact.parameter(),
                        &direction_x,
                        &direction_y,
                        range,
                        &self.data.policy,
                    ),
                    None => parallel.vector_tangent_cross_and_dot_signs(
                        contact.parameter(),
                        &direction_x,
                        &direction_y,
                        &self.data.policy,
                    ),
                };
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
            let chord_parameter = CurveRegionParameter2::from_algebraic_chord(chord_parameter);
            let parallel_parameter =
                CurveRegionParameter2::from_bezier(contact.parameter().clone());
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
        let (intersections, parameter_map) = match cusp
            .semicircle()
            .rational_intersections_with_parameter_map(rational, &self.data.policy)
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
            BezierAlgebraicCuspSemicircleRationalIntersections2::Contacts(contacts) => {
                let mut retained = Vec::with_capacity(contacts.len());
                for contact in contacts {
                    let cusp_parameter =
                        cusp_contact_parameter(contact.location).unwrap_or_else(|| {
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
                            CurveRegionParameter2::from_algebraic_cusp(cusp_parameter),
                            CurveRegionParameter2::from_bezier(contact.other_parameter),
                        )
                    } else {
                        (
                            CurveRegionParameter2::from_bezier(contact.other_parameter),
                            CurveRegionParameter2::from_algebraic_cusp(cusp_parameter),
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
                    overlaps: Vec::new(),
                    blockers: Vec::new(),
                })
            }
            BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiberContacts(
                contacts,
            ) => Ok(selected_fiber_cusp_contacts_result(contacts, cusp_is_first)),
            BezierAlgebraicCuspSemicircleRationalIntersections2::Overlaps(overlaps) => {
                Ok(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: overlaps
                        .into_iter()
                        .map(|overlap| {
                            let cusp_range = CurveRegionParameterRange2::new_validated(
                                CurveRegionParameter2::from_algebraic_cusp(
                                    overlap.cusp_start_parameter(),
                                ),
                                CurveRegionParameter2::from_algebraic_cusp(
                                    overlap.cusp_end_parameter(),
                                ),
                            );
                            let other_range = CurveRegionParameterRange2::from_bezier_range(
                                overlap.other_range().clone(),
                            );
                            let (first_range, second_range) = if cusp_is_first {
                                (cusp_range, other_range)
                            } else {
                                (other_range, cusp_range)
                            };
                            RegionPairOverlap {
                                source: Some(RegionPairOverlapSource::AlgebraicCuspMapped(
                                    overlap.clone(),
                                )),
                                first_range,
                                second_range,
                                orientation: overlap.orientation(),
                            }
                        })
                        .collect(),
                    blockers: Vec::new(),
                })
            }
            BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiberOverlaps(
                overlaps,
            ) => Ok(selected_fiber_cusp_overlaps_result(overlaps, cusp_is_first)),
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
                match cusp
                    .semicircle()
                    .chord_contact_tangent_topology(
                        &contact.cusp_parameter,
                        chord,
                        &self.data.policy,
                    )
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
            let (first_parameter, second_parameter) = if cusp_is_first {
                (
                    CurveRegionParameter2::from_algebraic_cusp(contact.cusp_parameter),
                    CurveRegionParameter2::from_algebraic_chord(contact.chord_parameter),
                )
            } else {
                (
                    CurveRegionParameter2::from_algebraic_chord(contact.chord_parameter),
                    CurveRegionParameter2::from_algebraic_cusp(contact.cusp_parameter),
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

    /// Replays a certified tangent support when a circle and chord from
    /// different arrangement components retain the same endpoint. Unlike an
    /// authored adjacent vertex, this contact remains part of the arrangement
    /// evidence; only the general circle/line solve is bypassed.
    fn certified_cusp_chord_endpoint_contact(
        &self,
        cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
        chord: &crate::BezierAlgebraicChord2,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleRetainedChordContact2>>>
    {
        for cusp_at_start in [true, false] {
            let point = match cusp.endpoint_point_evidence(cusp_at_start, &self.data.policy)? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) | Classification::Uncertain(_) => continue,
            };
            for (chord_at_start, endpoint) in [(true, chord.start()), (false, chord.end())] {
                if !point.shares_storage(endpoint)
                    && point.same_point(endpoint, &self.data.policy)
                        != Classification::Decided(true)
                    && endpoint.same_point(&point, &self.data.policy)
                        != Classification::Decided(true)
                {
                    continue;
                }
                if cusp.certified_adjacent_chord_is_endpoint_only(
                    chord,
                    cusp_at_start,
                    &self.data.policy,
                )? != Classification::Decided(true)
                {
                    continue;
                }
                return Ok(Classification::Decided(Some(
                    BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                        cusp_parameter: cusp.endpoint_parameter(cusp_at_start).clone(),
                        chord_parameter: if chord_at_start {
                            chord.start_parameter()
                        } else {
                            chord.end_parameter()
                        },
                        point,
                        tangent_cross_sign: RealSign::Zero,
                    },
                )));
            }
        }
        Ok(Classification::Decided(None))
    }

    fn pair_result(&self, pair: &RegionCarrierPair) -> ExactCurveResult<RegionPairResult> {
        let first = &self.data.carriers[pair.first_carrier_index];
        let second = &self.data.carriers[pair.second_carrier_index];
        match &pair.context {
            RegionCarrierPairContext::Bezier(context) => {
                let result = context.result_view()?;
                Ok(RegionPairResult {
                    contacts: result
                        .contacts()
                        .iter()
                        .map(RegionPairContactEvidence::from_bezier)
                        .collect(),
                    overlaps: result
                        .overlaps()
                        .iter()
                        .cloned()
                        .map(|source| RegionPairOverlap {
                            first_range: CurveRegionParameterRange2::from_bezier_range(
                                source.first_range().clone(),
                            ),
                            second_range: CurveRegionParameterRange2::from_bezier_range(
                                source.second_range().clone(),
                            ),
                            orientation: source.orientation(),
                            source: Some(RegionPairOverlapSource::Bezier(source)),
                        })
                        .collect(),
                    blockers: result
                        .blockers()
                        .iter()
                        .cloned()
                        .map(RegionPairBlocker::Bezier)
                        .collect(),
                })
            }
            RegionCarrierPairContext::BezierSelf => {
                let result = match self
                    .bezier_self_intersections(first.geometry.bezier())
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
                let contact_evidence = |contact: &crate::RationalBezierIntersectionContact2| {
                    RegionPairContactEvidence::direct_bezier(
                        contact.first_parameter().clone(),
                        contact.second_parameter().clone(),
                        Some(contact.point().clone()),
                        contact.is_certified_transverse(),
                        contact.tangent_cross_sign(),
                    )
                };
                let (contacts, blockers) = match result {
                    RationalBezierIntersectionContacts2::NoIntersection => (Vec::new(), Vec::new()),
                    RationalBezierIntersectionContacts2::Contacts(contacts) => {
                        (contacts.iter().map(contact_evidence).collect(), Vec::new())
                    }
                    RationalBezierIntersectionContacts2::Incomplete { contacts, .. } => (
                        contacts.iter().map(contact_evidence).collect(),
                        vec![RegionPairBlocker::IncompleteReplay],
                    ),
                    RationalBezierIntersectionContacts2::ContactsAndOverlap {
                        contacts, ..
                    } => (
                        contacts.iter().map(contact_evidence).collect(),
                        vec![RegionPairBlocker::Uncertain(UncertaintyReason::Boundary)],
                    ),
                    RationalBezierIntersectionContacts2::Overlap(_)
                    | RationalBezierIntersectionContacts2::DegenerateResultant => (
                        Vec::new(),
                        vec![RegionPairBlocker::Uncertain(UncertaintyReason::Boundary)],
                    ),
                };
                Ok(RegionPairResult {
                    contacts,
                    overlaps: Vec::new(),
                    blockers,
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
                let retained_range = match (
                    parallel_carrier.start.as_bezier_parameter(),
                    parallel_carrier.end.as_bezier_parameter(),
                ) {
                    (Some(start), Some(end)) => Some(BezierParameterRange2::new_validated(
                        start.clone(),
                        end.clone(),
                    )),
                    _ => None,
                };
                // Source-cusp branches retain one-sided endpoint point
                // evidence and must enter the range-aware common kernel before
                // the older circle shortcut, whose authored hodograph is
                // undefined at that endpoint. Ordinary carriers keep that cheap
                // interaction fast path. The line kernel is range-aware for
                // every retained subcarrier: a regular sibling of a cusp branch
                // need not itself own a selected endpoint, while its unsplit
                // authored source still has the singular hodograph.
                if parallel_carrier.selected_fiber_endpoint_points.is_none() {
                    match self.parallel_arc_pair_result(parallel, curve, *parallel_is_first)? {
                        Classification::Decided(Some(result)) => return Ok(result),
                        Classification::Decided(None) | Classification::Uncertain(_) => {}
                    }
                }
                let regular_range = retained_range
                    .as_ref()
                    .cloned()
                    .map(CurveRegionParameterRange2::from_bezier_range);
                match self.parallel_line_pair_result(
                    pair,
                    parallel,
                    parallel_index,
                    curve,
                    *parallel_is_first,
                    regular_range.as_ref(),
                )? {
                    Classification::Decided(Some(result)) => return Ok(result),
                    Classification::Decided(None) | Classification::Uncertain(_) => {}
                }
                let rational = RationalBezier2::try_from_subcurve(curve)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
                let intersections = match retained_range.as_ref() {
                    Some(range) => {
                        parallel.intersections_on_regular_range(&rational, range, &self.data.policy)
                    }
                    None => parallel.intersections(&rational, &self.data.policy),
                };
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
                let overlaps = result
                    .overlaps()
                    .iter()
                    .flat_map(|overlap| {
                        let (first_range, second_range) = if *parallel_is_first {
                            (
                                overlap.first_range().clone(),
                                overlap.second_range().clone(),
                            )
                        } else {
                            (
                                overlap.second_range().clone(),
                                overlap.first_range().clone(),
                            )
                        };
                        parameter_component_region_overlap_sources(
                            result.component_overlaps(),
                            overlap,
                            !*parallel_is_first,
                        )
                        .into_iter()
                        .map(move |source| RegionPairOverlap {
                            source,
                            first_range: CurveRegionParameterRange2::from_bezier_range(
                                first_range.clone(),
                            ),
                            second_range: CurveRegionParameterRange2::from_bezier_range(
                                second_range.clone(),
                            ),
                            orientation: overlap.orientation(),
                        })
                    })
                    .collect();
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
            | RegionCarrierPairContext::ParallelSameImage
            | RegionCarrierPairContext::ParallelSelf => {
                if !matches!(pair.context, RegionCarrierPairContext::ParallelSelf)
                    && (self.parallel_pair_is_coordinate_disjoint(pair)
                        || self.adjacent_parallel_pair_is_endpoint_only(pair))
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
                let intersection = match &pair.context {
                    RegionCarrierPairContext::ParallelPair
                    | RegionCarrierPairContext::ParallelSameImage => {
                        // The pair kernel saturates the identity component and
                        // replays every residual off-diagonal contact.  Adding
                        // the unordered self result would publish each fitting
                        // crossing twice.
                        match (
                            first.start.as_bezier_parameter(),
                            first.end.as_bezier_parameter(),
                            second.start.as_bezier_parameter(),
                            second.end.as_bezier_parameter(),
                        ) {
                            (
                                Some(first_start),
                                Some(first_end),
                                Some(second_start),
                                Some(second_end),
                            ) => parallel.parallel_intersections_on_regular_ranges(
                                second.geometry.parallel(),
                                &BezierParameterRange2::new_validated(
                                    first_start.clone(),
                                    first_end.clone(),
                                ),
                                &BezierParameterRange2::new_validated(
                                    second_start.clone(),
                                    second_end.clone(),
                                ),
                                &self.data.policy,
                            ),
                            _ => parallel.parallel_intersections(
                                second.geometry.parallel(),
                                &self.data.policy,
                            ),
                        }
                    }
                    RegionCarrierPairContext::ParallelSelf => {
                        self.parallel_self_intersections(parallel)
                    }
                    RegionCarrierPairContext::Bezier(_)
                    | RegionCarrierPairContext::BezierSelf
                    | RegionCarrierPairContext::AlgebraicChordPair
                    | RegionCarrierPairContext::CuspChord { .. }
                    | RegionCarrierPairContext::ParallelRational { .. }
                    | RegionCarrierPairContext::CuspRational { .. }
                    | RegionCarrierPairContext::CuspParallel { .. }
                    | RegionCarrierPairContext::CuspPair => unreachable!(),
                };
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
                        RegionPairContactEvidence::direct_bezier(
                            contact.first_parameter().clone(),
                            contact.second_parameter().clone(),
                            None,
                            contact.is_certified_transverse(),
                            contact.tangent_cross_sign(),
                        )
                    })
                    .collect();
                let overlaps = result
                    .overlaps()
                    .iter()
                    .flat_map(|overlap| {
                        parameter_component_region_overlap_sources(
                            result.component_overlaps(),
                            overlap,
                            false,
                        )
                        .into_iter()
                        .map(move |source| RegionPairOverlap {
                            source,
                            first_range: CurveRegionParameterRange2::from_bezier_range(
                                overlap.first_range().clone(),
                            ),
                            second_range: CurveRegionParameterRange2::from_bezier_range(
                                overlap.second_range().clone(),
                            ),
                            orientation: overlap.orientation(),
                        })
                    })
                    .collect();
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
                            first.geometry.algebraic_cusp(),
                            pair.first_carrier_index,
                            match &second.geometry {
                                RegionCarrierGeometry::AlgebraicChord(chord) => chord,
                                _ => unreachable!("cusp/chord dispatch retained its chord"),
                            },
                            pair.second_carrier_index,
                        )
                    } else {
                        (
                            second.geometry.algebraic_cusp(),
                            pair.second_carrier_index,
                            match &first.geometry {
                                RegionCarrierGeometry::AlgebraicChord(chord) => chord,
                                _ => unreachable!("chord/cusp dispatch retained its chord"),
                            },
                            pair.first_carrier_index,
                        )
                    };
                    let mut certified_chord_endpoint_incidence = None;
                    if let Some((first_at_start, second_at_start)) =
                        self.authored_carrier_shared_endpoints(pair)
                    {
                        let cusp_at_start = if *cusp_is_first {
                            first_at_start
                        } else {
                            second_at_start
                        };
                        if cusp.certified_tangent_endpoint(cusp_at_start) {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-circle-chord-pair",
                                "adjacent-authored-tangent",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        if cusp
                            .authored_adjacent_chord_is_structurally_endpoint_only(
                                chord,
                                cusp_at_start,
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(chord_index, cause))?
                        {
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
                    if certified_chord_endpoint_incidence.is_none()
                        && let Classification::Decided(Some(contact)) = self
                            .certified_cusp_chord_endpoint_contact(cusp, chord)
                            .map_err(|cause| self.invalid(chord_index, cause))?
                    {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-circle-chord-pair",
                            "retained-nonadjacent-endpoint-tangent",
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
                    let BezierAlgebraicCuspSemicircleRetainedChordIntersections2::Contacts(
                        mut contacts,
                    ) = intersections
                    else {
                        return Ok(RegionPairResult::empty());
                    };
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
            RegionCarrierPairContext::AlgebraicChordPair => {
                {
                    let (chord, chord_index, other, other_index) =
                        match (&first.geometry, &second.geometry) {
                            (RegionCarrierGeometry::AlgebraicChord(chord), other) => (
                                chord,
                                pair.first_carrier_index,
                                other,
                                pair.second_carrier_index,
                            ),
                            (other, RegionCarrierGeometry::AlgebraicChord(chord)) => (
                                chord,
                                pair.second_carrier_index,
                                other,
                                pair.first_carrier_index,
                            ),
                            _ => unreachable!("an algebraic-chord pair retains one chord"),
                        };
                    if carrier_refined_bounds_decided_disjoint(first, second, &self.data.policy)
                        .map_err(|cause| self.invalid(chord_index, cause))?
                    {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-pair",
                            "refined-carrier-bounds-disjoint",
                        );
                        return Ok(RegionPairResult::empty());
                    }
                    if let RegionCarrierGeometry::AlgebraicChord(other_chord) = other {
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
                                            CurveRegionParameter2::from_algebraic_chord(
                                                contact.first_parameter().clone(),
                                            ),
                                            CurveRegionParameter2::from_algebraic_chord(
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
                                        RegionPairOverlap {
                                            source: None,
                                            first_range: CurveRegionParameterRange2::new_validated(
                                                CurveRegionParameter2::from_algebraic_chord(
                                                    first_start.clone(),
                                                ),
                                                CurveRegionParameter2::from_algebraic_chord(
                                                    first_end.clone(),
                                                ),
                                            ),
                                            second_range: CurveRegionParameterRange2::new_validated(
                                                CurveRegionParameter2::from_algebraic_chord(
                                                    second_start.clone(),
                                                ),
                                                CurveRegionParameter2::from_algebraic_chord(
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
                    if let RegionCarrierGeometry::Bezier(curve) = other {
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
                            && other_carrier
                                .geometry
                                .has_certified_injective_image(&self.data.policy)
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
                            match chord
                                .has_non_collinear_support_with_exact_line(&line, &self.data.policy)
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
                        if let Some(result) = self
                            .algebraic_chord_shared_image_endpoint_pair_result(
                                pair,
                                chord,
                                chord_index,
                                &rational,
                                other_index,
                            )?
                        {
                            return Ok(result);
                        }
                        let shared_source_parameter =
                            if let Some(chord_precedes_other) = chord_precedes_other {
                                let shared_parameter = if chord_precedes_other {
                                    carrier_traversal_start(other_carrier)
                                } else {
                                    carrier_traversal_end(other_carrier)
                                };
                                shared_parameter.as_bezier_parameter()
                            } else {
                                None
                            };
                        if let Some(result) = self.algebraic_chord_rational_pair_result(
                            pair,
                            chord,
                            chord_index,
                            &rational,
                            shared_source_parameter,
                        )? {
                            return Ok(result);
                        }
                    }
                    if let RegionCarrierGeometry::AnalyticParallel(parallel) = other {
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
                        first.geometry.algebraic_cusp(),
                        second.geometry.bezier(),
                        second,
                        pair.second_carrier_index,
                    )
                } else {
                    (
                        second.geometry.algebraic_cusp(),
                        first.geometry.bezier(),
                        first,
                        pair.first_carrier_index,
                    )
                };
                if let Classification::Decided(bounds) = curve_carrier.bounds.get_or_init(|| {
                    curve_carrier
                        .geometry
                        .certified_outer_bounds(&self.data.policy)
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
                self.algebraic_cusp_rational_pair_result(pair, cusp, &rational, *cusp_is_first)
            }
            RegionCarrierPairContext::CuspParallel { cusp_is_first } => {
                let (cusp, parallel, parallel_carrier, parallel_index) = if *cusp_is_first {
                    (
                        first.geometry.algebraic_cusp(),
                        second.geometry.parallel(),
                        second,
                        pair.second_carrier_index,
                    )
                } else {
                    (
                        second.geometry.algebraic_cusp(),
                        first.geometry.parallel(),
                        first,
                        pair.first_carrier_index,
                    )
                };
                let parallel_range = match (
                    parallel_carrier.start.as_bezier_parameter(),
                    parallel_carrier.end.as_bezier_parameter(),
                ) {
                    (Some(start), Some(end)) => Some(BezierParameterRange2::new_validated(
                        start.clone(),
                        end.clone(),
                    )),
                    // A selected-fiber carrier still uses the analytic
                    // parallel's authored parameterization. Solve its complete
                    // support and let the generic carrier-range predicate
                    // retain only contacts inside the compact local interval.
                    _ => None,
                };
                // As above, approximate equality cannot choose a carrier
                // representation; it remains available to the delegated solve.
                if parallel_range.is_some()
                    && let Classification::Decided(Some(rational)) = parallel
                        .exact_rational_parallel_component(&CurveContext::STRICT)
                        .map_err(|cause| self.invalid(parallel_index, cause))?
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-parallel-pair",
                        "strict-rational-component",
                    );
                    return self.algebraic_cusp_rational_pair_result(
                        pair,
                        cusp,
                        &rational,
                        *cusp_is_first,
                    );
                }
                let intersections = match match parallel_range.as_ref() {
                    Some(range) => cusp.semicircle().parallel_intersections_in_range(
                        parallel,
                        range,
                        &self.data.policy,
                    ),
                    None => cusp
                        .semicircle()
                        .parallel_intersections(parallel, &self.data.policy),
                }
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
                let contacts = match intersections {
                    BezierAlgebraicCuspSemicircleParallelIntersections2::Contacts(contacts) => {
                        contacts
                    }
                    BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiberContacts(
                        contacts,
                    ) => {
                        return Ok(selected_fiber_cusp_contacts_result(
                            contacts,
                            *cusp_is_first,
                        ));
                    }
                    BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiberOverlaps(
                        overlaps,
                    ) => {
                        return Ok(selected_fiber_cusp_overlaps_result(
                            overlaps,
                            *cusp_is_first,
                        ));
                    }
                    BezierAlgebraicCuspSemicircleParallelIntersections2::Overlaps(overlaps) => {
                        let overlaps = overlaps
                            .into_iter()
                            .map(|overlap| {
                                let cusp_range = CurveRegionParameterRange2::new_validated(
                                    CurveRegionParameter2::from_algebraic_cusp(
                                        overlap.cusp_start_parameter(),
                                    ),
                                    CurveRegionParameter2::from_algebraic_cusp(
                                        overlap.cusp_end_parameter(),
                                    ),
                                );
                                let parallel_range = CurveRegionParameterRange2::from_bezier_range(
                                    overlap.other_range().clone(),
                                );
                                let (first_range, second_range) = if *cusp_is_first {
                                    (cusp_range, parallel_range)
                                } else {
                                    (parallel_range, cusp_range)
                                };
                                RegionPairOverlap {
                                    source: Some(RegionPairOverlapSource::AlgebraicCuspMapped(
                                        overlap.clone(),
                                    )),
                                    first_range,
                                    second_range,
                                    orientation: overlap.orientation(),
                                }
                            })
                            .collect();
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps,
                            blockers: Vec::new(),
                        });
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
                let parameter_map = if contacts.iter().any(|contact| {
                    contact.location == BezierAlgebraicCuspSemicircleContactLocation2::Interior
                }) {
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
                    let cusp_parameter =
                        cusp_contact_parameter(contact.location).unwrap_or_else(|| {
                            parameter_map
                                .as_ref()
                                .expect(
                                    "an interior cusp/parallel contact retains its parameter map",
                                )
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
                            CurveRegionParameter2::from_algebraic_cusp(cusp_parameter),
                            CurveRegionParameter2::from_bezier(contact.parallel_parameter),
                        )
                    } else {
                        (
                            CurveRegionParameter2::from_bezier(contact.parallel_parameter),
                            CurveRegionParameter2::from_algebraic_cusp(cusp_parameter),
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
                    overlaps: Vec::new(),
                    blockers: Vec::new(),
                })
            }
            RegionCarrierPairContext::CuspPair => {
                let first_cusp = first.geometry.algebraic_cusp();
                let second_cusp = second.geometry.algebraic_cusp();
                if let Some((first_at_start, second_at_start)) =
                    self.authored_carrier_shared_endpoints(pair)
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
                            CurveRegionParameter2::from_algebraic_cusp(first_parameter),
                            CurveRegionParameter2::from_algebraic_cusp(second_parameter),
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
                                CurveRegionParameter2::from_algebraic_cusp(
                                    parameter_map.first_contact_parameter(&contact),
                                ),
                                CurveRegionParameter2::from_algebraic_cusp(
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
                            let first_parameter = cusp_contact_parameter(contact.first_location)
                                .expect("an endpoint contact names a first cusp endpoint");
                            let second_parameter = cusp_contact_parameter(contact.second_location)
                                .expect("an endpoint contact names a second cusp endpoint");
                            retained.push(RegionPairContactEvidence::direct(
                                CurveRegionParameter2::from_algebraic_cusp(first_parameter),
                                CurveRegionParameter2::from_algebraic_cusp(second_parameter),
                                None,
                                false,
                                Some(RealSign::Zero),
                            ));
                        }
                    }
                    BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(overlap) => {
                        overlaps.push(RegionPairOverlap {
                            source: Some(RegionPairOverlapSource::AlgebraicCusp(overlap.clone())),
                            first_range: CurveRegionParameterRange2::new_validated(
                                CurveRegionParameter2::from_algebraic_cusp(
                                    overlap.first_start_parameter(),
                                ),
                                CurveRegionParameter2::from_algebraic_cusp(
                                    overlap.first_end_parameter(),
                                ),
                            ),
                            second_range: CurveRegionParameterRange2::new_validated(
                                CurveRegionParameter2::from_algebraic_cusp(
                                    overlap.second_start_parameter(),
                                ),
                                CurveRegionParameter2::from_algebraic_cusp(
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
        let (
            RegionCarrierGeometry::AnalyticParallel(first_parallel),
            RegionCarrierGeometry::AnalyticParallel(second_parallel),
        ) = (&first.geometry, &second.geometry)
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
            if !first_parallel
                .regular_fragment_has_certified_injective_axis_on(axis, &self.data.policy)
                || !second_parallel
                    .regular_fragment_has_certified_injective_axis_on(axis, &self.data.policy)
            {
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
        let (
            RegionCarrierGeometry::AnalyticParallel(first_parallel),
            RegionCarrierGeometry::AnalyticParallel(second_parallel),
        ) = (&first.geometry, &second.geometry)
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
            if !first_parallel
                .regular_fragment_has_certified_injective_axis_on(axis, &self.data.policy)
                || !second_parallel
                    .regular_fragment_has_certified_injective_axis_on(axis, &self.data.policy)
            {
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

    fn clipped_overlap_ranges(
        &self,
        pair: &RegionCarrierPair,
        overlap: &RegionPairOverlap,
    ) -> ExactCurveResult<Option<(CurveRegionParameterRange2, CurveRegionParameterRange2)>> {
        let first_carrier = &self.data.carriers[pair.first_carrier_index];
        let second_carrier = &self.data.carriers[pair.second_carrier_index];
        let first_intersects =
            ranges_intersect(&overlap.first_range, first_carrier, &self.data.policy)?;
        let second_intersects =
            ranges_intersect(&overlap.second_range, second_carrier, &self.data.policy)?;
        if !first_intersects || !second_intersects {
            return Ok(None);
        }
        if range_inside_carrier(&overlap.first_range, first_carrier, &self.data.policy)?
            && range_inside_carrier(&overlap.second_range, second_carrier, &self.data.policy)?
        {
            return Ok(Some((
                overlap.first_range.clone(),
                overlap.second_range.clone(),
            )));
        }
        if let Some(RegionPairOverlapSource::ParameterComponent { source, swapped }) =
            overlap.source.as_ref()
        {
            let first_range = CurveRegionParameterRange2::new_validated(
                first_carrier.start.clone(),
                first_carrier.end.clone(),
            );
            let second_range = CurveRegionParameterRange2::new_validated(
                second_carrier.start.clone(),
                second_carrier.end.clone(),
            );
            let clipped = if *swapped {
                source
                    .clipped_curve_region_ranges(&second_range, &first_range, &self.data.policy)
                    .map(|classification| classification.map(|ranges| ranges.map(|(a, b)| (b, a))))
            } else {
                source.clipped_curve_region_ranges(&first_range, &second_range, &self.data.policy)
            }
            .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
            return match clipped {
                Classification::Decided(Some(ranges)) => Ok(Some(ranges)),
                Classification::Decided(None) => Ok(None),
                Classification::Uncertain(reason) => {
                    Err(self.blocked(pair.first_carrier_index, reason))
                }
            };
        }
        if let Some(RegionPairOverlapSource::AlgebraicCusp(source)) = overlap.source.as_ref() {
            let first_range = CurveRegionParameterRange2::new_validated(
                first_carrier.start.clone(),
                first_carrier.end.clone(),
            );
            let second_range = CurveRegionParameterRange2::new_validated(
                second_carrier.start.clone(),
                second_carrier.end.clone(),
            );
            return match source
                .clipped_curve_region_ranges(&first_range, &second_range, &self.data.policy)
                .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
            {
                Classification::Decided(ranges) => Ok(ranges),
                Classification::Uncertain(reason) => {
                    Err(self.blocked(pair.first_carrier_index, reason))
                }
            };
        }
        if let Some(RegionPairOverlapSource::AlgebraicCuspMapped(source)) = overlap.source.as_ref()
        {
            debug_assert_eq!(source.orientation(), overlap.orientation);
            return self.clip_cusp_mapped_overlap(
                pair,
                overlap,
                RegionCuspMappedOverlapRef::Bezier(source),
            );
        }
        if let Some(RegionPairOverlapSource::AlgebraicCuspSelectedFiberMapped(source)) =
            overlap.source.as_ref()
        {
            debug_assert_eq!(source.orientation(), overlap.orientation);
            return self.clip_cusp_mapped_overlap(
                pair,
                overlap,
                RegionCuspMappedOverlapRef::Selected(source),
            );
        }
        if let Some(RegionPairOverlapSource::AlgebraicChordRational(source)) =
            overlap.source.as_ref()
        {
            debug_assert_eq!(source.orientation(), overlap.orientation);
            return self.clip_algebraic_chord_rational_overlap(pair, overlap, source);
        }
        let Some((first_start, first_end)) = overlap.first_range.as_bezier_parameters() else {
            return Err(self.blocked(pair.first_carrier_index, UncertaintyReason::Unsupported));
        };
        let Some((second_start, second_end)) = overlap.second_range.as_bezier_parameters() else {
            return Err(self.blocked(pair.second_carrier_index, UncertaintyReason::Unsupported));
        };
        let first_range =
            BezierParameterRange2::new_validated(first_start.clone(), first_end.clone());
        let second_range =
            BezierParameterRange2::new_validated(second_start.clone(), second_end.clone());
        let correspondence = overlap.source.as_ref().and_then(|source| match source {
            RegionPairOverlapSource::Bezier(source) => source.parameter_correspondence(),
            RegionPairOverlapSource::ParameterComponent { .. } => None,
            RegionPairOverlapSource::AlgebraicChordRational(_) => None,
            RegionPairOverlapSource::AlgebraicCusp(_)
            | RegionPairOverlapSource::AlgebraicCuspMapped(_) => None,
            RegionPairOverlapSource::AlgebraicCuspSelectedFiberMapped(_) => None,
        });
        if let Some(correspondence) = correspondence {
            if let Some(reversed) = correspondence.projective_reversal() {
                return match clip_aligned_parameter_overlap(
                    &first_range,
                    &second_range,
                    reversed,
                    first_carrier,
                    second_carrier,
                    &self.data.policy,
                )? {
                    CarrierOverlapClip::Matched(ranges) => Ok(ranges),
                    CarrierOverlapClip::Unmatched => {
                        Err(self.blocked(pair.first_carrier_index, UncertaintyReason::Predicate))
                    }
                };
            }
            return clip_corresponding_parameter_overlap(
                &first_range,
                &second_range,
                correspondence,
                first_carrier,
                second_carrier,
                &self.data.policy,
            );
        }
        match clip_projectively_aligned_parameter_overlap(
            &first_range,
            &second_range,
            overlap.orientation,
            first_carrier,
            second_carrier,
            &self.data.policy,
        )? {
            CarrierOverlapClip::Matched(ranges) => Ok(ranges),
            CarrierOverlapClip::Unmatched if overlap.source.is_none() => {
                let first = match first_carrier
                    .geometry
                    .exact_rational_component(&self.data.policy)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    Classification::Decided(Some(curve)) => curve,
                    Classification::Decided(None) => {
                        return Err(
                            self.blocked(pair.first_carrier_index, UncertaintyReason::Unsupported)
                        );
                    }
                    Classification::Uncertain(reason) => {
                        return Err(self.blocked(pair.first_carrier_index, reason));
                    }
                };
                let second = match second_carrier
                    .geometry
                    .exact_rational_component(&self.data.policy)
                    .map_err(|cause| self.invalid(pair.second_carrier_index, cause))?
                {
                    Classification::Decided(Some(curve)) => curve,
                    Classification::Decided(None) => {
                        return Err(
                            self.blocked(pair.second_carrier_index, UncertaintyReason::Unsupported)
                        );
                    }
                    Classification::Uncertain(reason) => {
                        return Err(self.blocked(pair.second_carrier_index, reason));
                    }
                };
                let raw_overlap = RationalBezierIntersectionOverlap2::from_certified_parameters(
                    first_range.start().clone(),
                    first_range.end().clone(),
                    second_range.start().clone(),
                    second_range.end().clone(),
                    overlap.orientation,
                    [true, true],
                );
                let correspondence = RationalBezierOverlapParameterCorrespondence2::for_overlap(
                    &first,
                    &second,
                    &raw_overlap,
                    &self.data.policy,
                );
                clip_corresponding_parameter_overlap(
                    &first_range,
                    &second_range,
                    &correspondence,
                    first_carrier,
                    second_carrier,
                    &self.data.policy,
                )
            }
            CarrierOverlapClip::Unmatched => {
                Err(self.blocked(pair.first_carrier_index, UncertaintyReason::Unsupported))
            }
        }
    }

    fn clip_algebraic_chord_rational_overlap(
        &self,
        pair: &RegionCarrierPair,
        overlap: &RegionPairOverlap,
        source: &BezierAlgebraicChordRationalOverlap2,
    ) -> ExactCurveResult<Option<(CurveRegionParameterRange2, CurveRegionParameterRange2)>> {
        let chord_is_first = overlap.first_range.start().is_algebraic_chord();
        if chord_is_first == overlap.second_range.start().is_algebraic_chord() {
            return Err(self.invalid(
                pair.first_carrier_index,
                CurveError::Topology(
                    "algebraic-chord overlap did not retain exactly one chord range".into(),
                ),
            ));
        }
        let (chord_carrier_index, source_carrier_index, source_overlap_range) = if chord_is_first {
            (
                pair.first_carrier_index,
                pair.second_carrier_index,
                &overlap.second_range,
            )
        } else {
            (
                pair.second_carrier_index,
                pair.first_carrier_index,
                &overlap.first_range,
            )
        };
        let source_carrier = &self.data.carriers[source_carrier_index];
        let (overlap_low, overlap_high) = ascending_range(source_overlap_range, &self.data.policy)?;
        let source_low = if decided_parameter_cmp(
            overlap_low,
            &source_carrier.start,
            &self.data.policy,
        )?
        .is_lt()
        {
            source_carrier.start.clone()
        } else {
            overlap_low.clone()
        };
        let source_high =
            if decided_parameter_cmp(overlap_high, &source_carrier.end, &self.data.policy)?.is_gt()
            {
                source_carrier.end.clone()
            } else {
                overlap_high.clone()
            };
        match decided_parameter_cmp(&source_low, &source_high, &self.data.policy)? {
            Ordering::Less => {}
            Ordering::Equal | Ordering::Greater => return Ok(None),
        }

        let map_source_parameter = |parameter: &CurveRegionParameter2| {
            let Some(parameter) = parameter.as_bezier_parameter() else {
                return Err(self.invalid(
                    source_carrier_index,
                    CurveError::Topology(
                        "non-Bezier cut reached an algebraic-chord/rational overlap".into(),
                    ),
                ));
            };
            match source
                .chord_parameter_at_source_parameter(parameter, &self.data.policy)
                .map_err(|cause| self.invalid(chord_carrier_index, cause))?
            {
                Classification::Decided(Some(parameter)) => {
                    Ok(CurveRegionParameter2::from_algebraic_chord(parameter))
                }
                Classification::Decided(None) => {
                    Err(self.blocked(chord_carrier_index, UncertaintyReason::Boundary))
                }
                Classification::Uncertain(reason) => Err(self.blocked(chord_carrier_index, reason)),
            }
        };
        let chord_at_source_low = map_source_parameter(&source_low)?;
        let chord_at_source_high = map_source_parameter(&source_high)?;
        let chord_order = decided_parameter_cmp(
            &chord_at_source_low,
            &chord_at_source_high,
            &self.data.policy,
        )?;
        let (chord_range, source_range, orientation) = match chord_order {
            Ordering::Less => (
                CurveRegionParameterRange2::new_validated(
                    chord_at_source_low,
                    chord_at_source_high,
                ),
                CurveRegionParameterRange2::new_validated(source_low, source_high),
                RationalBezierOverlapOrientation2::Same,
            ),
            Ordering::Greater => (
                CurveRegionParameterRange2::new_validated(
                    chord_at_source_high,
                    chord_at_source_low,
                ),
                CurveRegionParameterRange2::new_validated(source_high, source_low),
                RationalBezierOverlapOrientation2::Reversed,
            ),
            Ordering::Equal => {
                return Err(self.blocked(chord_carrier_index, UncertaintyReason::Boundary));
            }
        };
        if orientation != overlap.orientation {
            return Err(self.invalid(
                chord_carrier_index,
                CurveError::Topology(
                    "clipped algebraic-chord overlap changed parameter orientation".into(),
                ),
            ));
        }
        Ok(Some(if chord_is_first {
            (chord_range, source_range)
        } else {
            (source_range, chord_range)
        }))
    }

    fn clip_cusp_mapped_overlap(
        &self,
        pair: &RegionCarrierPair,
        overlap: &RegionPairOverlap,
        source: RegionCuspMappedOverlapRef<'_>,
    ) -> ExactCurveResult<Option<(CurveRegionParameterRange2, CurveRegionParameterRange2)>> {
        let first_is_cusp = overlap.first_range.start().is_algebraic_cusp();
        let second_is_cusp = overlap.second_range.start().is_algebraic_cusp();
        if first_is_cusp == second_is_cusp {
            return Err(self.invalid(
                pair.first_carrier_index,
                CurveError::Topology(
                    "mapped cusp overlap did not retain exactly one cusp parameter range".into(),
                ),
            ));
        }
        let (cusp_carrier, other_carrier, cusp_carrier_index) = if first_is_cusp {
            (
                &self.data.carriers[pair.first_carrier_index],
                &self.data.carriers[pair.second_carrier_index],
                pair.first_carrier_index,
            )
        } else {
            (
                &self.data.carriers[pair.second_carrier_index],
                &self.data.carriers[pair.first_carrier_index],
                pair.second_carrier_index,
            )
        };
        let range = |carrier: &RegionCarrier| {
            CurveRegionParameterRange2::new_validated(carrier.start.clone(), carrier.end.clone())
        };
        let clipped = source
            .clipped_curve_region_ranges(
                &range(cusp_carrier),
                &range(other_carrier),
                &self.data.policy,
            )
            .map_err(|cause| self.invalid(cusp_carrier_index, cause))?;
        match clipped {
            Classification::Decided(ranges) => Ok(ranges.map(|(cusp, other)| {
                if first_is_cusp {
                    (cusp, other)
                } else {
                    (other, cusp)
                }
            })),
            Classification::Uncertain(reason) => Err(self.blocked(cusp_carrier_index, reason)),
        }
    }

    fn build_split_topology(&self) -> ExactCurveResult<CurveRegionSplitTopology> {
        let mut events = vec![Vec::new(); self.data.carriers.len()];
        let mut contact_points = Vec::<ContactVertex>::new();
        let mut deferred_contact_matches = Vec::<(usize, usize, UncertaintyReason)>::new();
        let mut merge_vertices = Vec::new();
        let mut uncertain_contact_matches = Vec::new();
        let mut deferred_event_ordering = false;
        let mut next_topology_vertex = 0_usize;
        let mut contact_vertex_counts = Vec::<usize>::new();
        let mut transition_candidates = Vec::<Option<TransitionContactCandidate>>::new();
        let mut reclassification_vertices = Vec::<bool>::new();
        seed_loop_topology_vertices(&self.data.carriers, &mut events, &mut next_topology_vertex);
        contact_vertex_counts.resize(next_topology_vertex, 0);
        transition_candidates.resize(next_topology_vertex, None);
        reclassification_vertices.resize(next_topology_vertex, false);
        let mut overlaps = Vec::<CarrierOverlap>::new();
        for pair in &self.data.pairs {
            let result = self.pair_result(pair)?;
            if let Some(blocker) = result.blockers.first() {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-regularization-pair-blocker",
                    match &pair.context {
                        RegionCarrierPairContext::Bezier(_) => "bezier-pair",
                        RegionCarrierPairContext::ParallelRational {
                            parallel_is_first: true,
                        } => "parallel-rational",
                        RegionCarrierPairContext::ParallelRational {
                            parallel_is_first: false,
                        } => "rational-parallel",
                        RegionCarrierPairContext::ParallelPair => "parallel-pair",
                        RegionCarrierPairContext::ParallelSameImage => "parallel-same-image",
                        RegionCarrierPairContext::ParallelSelf => "parallel-self",
                        RegionCarrierPairContext::BezierSelf => "bezier-self",
                        RegionCarrierPairContext::AlgebraicChordPair => "algebraic-chord-pair",
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
                    RegionPairBlocker::Bezier(blocker) => match blocker.kind() {
                        crate::CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
                        crate::CurveIntersectionPairBlockerKind2::IncompleteReplay { .. } => {
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
                let first_existing = existing_event_vertex_if_decided(
                    &events[pair.first_carrier_index],
                    first_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
                let second_existing = existing_event_vertex_if_decided(
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
                for (existing_index, existing) in contact_points.iter().enumerate() {
                    if topology_vertex == Some(existing.topology_vertex) {
                        matching_contact_index.get_or_insert(existing_index);
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
                                RationalBezierIntersectionPointEvidence2::Algebraic(_),
                                RationalBezierIntersectionPointEvidence2::Exact(exact),
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
                            Some(RationalBezierIntersectionPointEvidence2::Algebraic(_))
                        ) && matches!(
                            contact.point(),
                            Some(RationalBezierIntersectionPointEvidence2::Exact(_))
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
                contact_points.push(ContactVertex {
                    point,
                    topology_vertex,
                    carrier_indices: [pair.first_carrier_index, pair.second_carrier_index],
                    parameters: [first_parameter.clone(), second_parameter.clone()],
                });
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
                            .then(|| [first_parameter.clone(), second_parameter.clone()]),
                    })
                } else {
                    None
                };
                deferred_event_ordering |= push_contact_carrier_event(
                    &mut events[pair.first_carrier_index],
                    first_parameter.clone(),
                    Some(topology_vertex),
                    &self.data.carriers[pair.first_carrier_index],
                    &self.data.policy,
                )?;
                deferred_event_ordering |= push_contact_carrier_event(
                    &mut events[pair.second_carrier_index],
                    second_parameter.clone(),
                    Some(topology_vertex),
                    &self.data.carriers[pair.second_carrier_index],
                    &self.data.policy,
                )?;
            }

            for overlap in &result.overlaps {
                let Some((mut first_range, mut second_range)) =
                    self.clipped_overlap_ranges(pair, overlap)?
                else {
                    continue;
                };
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
                let first_direction = decided_parameter_cmp(
                    &first_parameters[0],
                    &first_parameters[1],
                    &self.data.policy,
                )?;
                let second_direction = decided_parameter_cmp(
                    &second_parameters[0],
                    &second_parameters[1],
                    &self.data.policy,
                )?;
                if first_direction == Ordering::Equal || second_direction == Ordering::Equal {
                    return Err(
                        self.invalid(pair.first_carrier_index, CurveError::DegenerateOverlapRange)
                    );
                }
                let endpoints_correspond_in_order = (first_direction == second_direction)
                    == (overlap.orientation == RationalBezierOverlapOrientation2::Same);
                for index in [0_usize, 1] {
                    let second_index = if endpoints_correspond_in_order {
                        index
                    } else {
                        1 - index
                    };
                    let first_existing = existing_event_vertex_if_decided(
                        &events[pair.first_carrier_index],
                        &first_parameters[index],
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
                    let second_existing = existing_event_vertex_if_decided(
                        &events[pair.second_carrier_index],
                        &second_parameters[second_index],
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
                    transition_candidates[topology_vertex] = None;
                    reclassification_vertices[topology_vertex] = true;
                    first_endpoint_vertices[index] = topology_vertex;
                    second_endpoint_vertices[second_index] = topology_vertex;
                    first_parameters[index] = push_canonical_carrier_event(
                        &mut events[pair.first_carrier_index],
                        first_parameters[index].clone(),
                        Some(topology_vertex),
                        &self.data.carriers[pair.first_carrier_index],
                        &self.data.policy,
                    )?;
                    second_parameters[second_index] = push_canonical_carrier_event(
                        &mut events[pair.second_carrier_index],
                        second_parameters[second_index].clone(),
                        Some(topology_vertex),
                        &self.data.carriers[pair.second_carrier_index],
                        &self.data.policy,
                    )?;
                    if let Some(RegionPairOverlapSource::AlgebraicCuspSelectedFiberMapped(source)) =
                        overlap.source.as_ref()
                    {
                        let selected_parameter = first_parameters[index]
                            .as_selected_fiber()
                            .or_else(|| second_parameters[second_index].as_selected_fiber());
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
                            contact_points.push(ContactVertex {
                                point: Some(point),
                                topology_vertex,
                                carrier_indices: [
                                    pair.first_carrier_index,
                                    pair.second_carrier_index,
                                ],
                                parameters: [
                                    first_parameters[index].clone(),
                                    second_parameters[second_index].clone(),
                                ],
                            });
                        }
                    }
                }
                first_range = CurveRegionParameterRange2::new_validated(
                    first_parameters[0].clone(),
                    first_parameters[1].clone(),
                );
                second_range = CurveRegionParameterRange2::new_validated(
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
            for (carrier_index, range) in [
                (overlap.first_carrier_index, &overlap.first_range),
                (overlap.second_carrier_index, &overlap.second_range),
            ] {
                for parameter in [range.start(), range.end()] {
                    if let Some(vertex) =
                        existing_event_vertex(&events[carrier_index], parameter, &self.data.policy)?
                        && transition_candidates.get(vertex).is_some()
                    {
                        reclassification_vertices[vertex] = true;
                    }
                }
            }
        }
        if deferred_event_ordering {
            validate_carrier_event_separation(&events, &self.data.carriers, &self.data.policy)?;
        }

        let mut exact_contact_point_index_by_vertex = vec![usize::MAX; next_topology_vertex];
        for (contact_index, contact) in contact_points.iter().enumerate() {
            if matches!(
                contact.point,
                Some(RationalBezierIntersectionPointEvidence2::Exact(_))
            ) {
                exact_contact_point_index_by_vertex[contact.topology_vertex] = contact_index;
            }
        }
        let split_fragments = self
            .data
            .carriers
            .iter()
            .enumerate()
            .map(|(carrier_index, carrier)| {
                let result = split_carrier(
                    carrier,
                    &events[carrier_index],
                    &contact_points,
                    &exact_contact_point_index_by_vertex,
                    &self.data.policy,
                );
                result.map_err(|cause| self.invalid(carrier_index, cause))
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
        parameter: &CurveRegionParameter2,
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
        let topology = self.build_split_topology()?;
        let simple_loop_filled_side = self.certified_simple_single_loop_filled_side(&topology);
        let fragment_selection =
            self.regularized_fragment_actions(&topology, simple_loop_filled_side)?;
        let mut arrangement_fragments = Vec::new();
        let mut arrangement_directions = Vec::new();
        let mut arrangement_source_edge_ids = Vec::new();
        let mut source_edge_index = 0_usize;
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for (split_fragment_index, split) in splits.iter().enumerate() {
                let current_source_edge = source_edge_index;
                source_edge_index += 1;
                let source_range = split.fragment.curve_region_parameter_range();
                let (source_start, source_end) = (source_range.start(), source_range.end());
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
        );
        let traversal = match graph.traverse_retained_filled_left_faces_with_certified_successors(
            &certified_successors,
            &self.data.policy,
        ) {
            Classification::Decided(traversal) => traversal,
            Classification::Uncertain(reason) => return Err(self.blocked(0, reason)),
        };
        let mut region =
            match CurveRegion2::from_certified_retained_arrangement_traversal(&graph, &traversal) {
                Classification::Decided(region) => region,
                Classification::Uncertain(reason) => return Err(self.blocked(0, reason)),
            }
            .with_certified_regularized_filled_left_topology()
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
        let mut face_union = RegularizedFaceUnion::new(edge_count.saturating_mul(2));
        let mut vertex_sector_links = Vec::new();

        // An authored continuation with no transverse event preserves both
        // local face sectors exactly.
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
            let incoming = edge_index(incoming.0, incoming.1);
            let outgoing = edge_index(outgoing.0, outgoing.1);
            face_union.unite(left_face(incoming), left_face(outgoing));
            face_union.unite(right_face(incoming), right_face(outgoing));
        }

        let overlap_orientation_between_edges = |first_edge: usize,
                                                 second_edge: usize|
         -> ExactCurveResult<Option<bool>> {
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
                let reversed = (overlap.orientation == RationalBezierOverlapOrientation2::Reversed)
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

        // Exact crossing and tangent certificates fix every local face sector
        // without materializing the contact coordinate. A contact at an
        // authored carrier endpoint borrows the adjacent carrier branch.
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
                        if is_start { range.start() } else { range.end() },
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
            for [incoming_slot, outgoing_slot] in [[0, 1], [2, 3]] {
                match (branches[incoming_slot], branches[outgoing_slot]) {
                    (Some(incoming), None) => {
                        let incoming_source = edge_sources[incoming];
                        let candidates = incident.iter().filter_map(
                            |&(carrier_index, split_index, is_start)| {
                                if !is_start
                                    || !authored_successor(
                                        incoming_source,
                                        (carrier_index, split_index),
                                    )
                                {
                                    return None;
                                }
                                Some(edge_index(carrier_index, split_index))
                            },
                        );
                        let mut candidates = candidates.fuse();
                        if let Some(candidate) = candidates.next()
                            && candidates.next().is_none()
                        {
                            branches[outgoing_slot] = Some(candidate);
                        }
                    }
                    (None, Some(outgoing)) => {
                        let outgoing_source = edge_sources[outgoing];
                        let candidates = incident.iter().filter_map(
                            |&(carrier_index, split_index, is_start)| {
                                if is_start
                                    || !authored_successor(
                                        (carrier_index, split_index),
                                        outgoing_source,
                                    )
                                {
                                    return None;
                                }
                                Some(edge_index(carrier_index, split_index))
                            },
                        );
                        let mut candidates = candidates.fuse();
                        if let Some(candidate) = candidates.next()
                            && candidates.next().is_none()
                        {
                            branches[incoming_slot] = Some(candidate);
                        }
                    }
                    (Some(_), Some(_)) | (None, None) => {}
                }
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
                    unite_regularized_vertex_sectors(
                        &mut face_union,
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
                    unite_regularized_vertex_sectors(
                        &mut face_union,
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
                unite_regularized_vertex_sectors(
                    &mut face_union,
                    &mut vertex_sector_links,
                    vertex,
                    &sector_pairs,
                );
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
            unite_regularized_vertex_sectors(
                &mut face_union,
                &mut vertex_sector_links,
                vertex,
                &sector_pairs,
            );
        }

        let mut face_roots = topology
            .split_fragments
            .iter()
            .enumerate()
            .map(|(carrier_index, splits)| {
                splits
                    .iter()
                    .enumerate()
                    .map(|(split_index, _)| {
                        let edge = edge_index(carrier_index, split_index);
                        [
                            face_union.find(left_face(edge)),
                            face_union.find(right_face(edge)),
                        ]
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        // Compress roots once more after the last union. This also makes the
        // sparse root indices stable for the adjacency lists below.
        for roots in face_roots.iter_mut().flatten() {
            roots[0] = face_union.find(roots[0]);
            roots[1] = face_union.find(roots[1]);
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
                    let (own_range, other_carrier_index) =
                        if overlap.first_carrier_index == carrier_index {
                            (Some(&overlap.first_range), overlap.second_carrier_index)
                        } else if overlap.second_carrier_index == carrier_index {
                            (Some(&overlap.second_range), overlap.first_carrier_index)
                        } else {
                            (None, usize::MAX)
                        };
                    let Some(own_range) = own_range else {
                        continue;
                    };
                    let contains =
                        match carrier_overlap_split_interval(overlap, carrier_index, splits) {
                            Some(interval) => interval.contains(&split_index),
                            None => range_contains_fragment(
                                own_range,
                                range.start(),
                                range.end(),
                                &self.data.policy,
                            )?,
                        };
                    if contains {
                        edge_overlapped[carrier_index][split_index] = true;
                        if other_carrier_index < carrier_index {
                            edge_owns_overlap[carrier_index][split_index] = false;
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
                                 overlap_range: &CurveRegionParameterRange2|
             -> ExactCurveResult<Vec<usize>> {
                let mut edges = Vec::new();
                for (split_index, split) in
                    topology.split_fragments[carrier_index].iter().enumerate()
                {
                    let range = split.fragment.curve_region_parameter_range();
                    let contains = match carrier_overlap_split_interval(
                        overlap,
                        carrier_index,
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
            let first_edges = collect_edges(overlap.first_carrier_index, &overlap.first_range)?;
            let second_edges = collect_edges(overlap.second_carrier_index, &overlap.second_range)?;
            let first_carrier = &self.data.carriers[overlap.first_carrier_index];
            let second_carrier = &self.data.carriers[overlap.second_carrier_index];
            let reversed = (overlap.orientation == RationalBezierOverlapOrientation2::Reversed)
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
                if reversed {
                    face_union.unite(left_face(first_edge), right_face(second_edge));
                    face_union.unite(right_face(first_edge), left_face(second_edge));
                } else {
                    face_union.unite(left_face(first_edge), left_face(second_edge));
                    face_union.unite(right_face(first_edge), right_face(second_edge));
                }
            }
        }

        // Overlap unions were added after the transverse-sector pass; refresh
        // every sparse face root before constructing winding equations.
        for (carrier_index, roots) in face_roots.iter_mut().enumerate() {
            for (split_index, roots) in roots.iter_mut().enumerate() {
                let edge = edge_index(carrier_index, split_index);
                roots[0] = face_union.find(left_face(edge));
                roots[1] = face_union.find(right_face(edge));
            }
        }

        let mut face_adjacency = vec![Vec::new(); edge_count.saturating_mul(2)];
        let mut winding_jumps = Vec::new();
        let mut edge_overlap_grouped = vec![false; edge_count];
        let mut overlap_orientation = vec![None; edge_count];
        let mut face_equations_valid = true;
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
                continue;
            }
            let reference = members[0].0;
            let [left, right] = face_roots[edge_sources[reference].0][edge_sources[reference].1];
            if left == right {
                // A contact-sector union may conservatively collapse both
                // sides of a same-image span at a multi-branch endpoint. A
                // nonzero aggregate winding jump cannot be a self-loop, so
                // disable the derived face accelerator below and retain the
                // authoritative per-fragment geometric classification path.
                face_equations_valid = false;
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
                    // well as an overlap edge.  The sparse face equations are
                    // then unusable, but the per-fragment geometric
                    // classifier below remains authoritative and exact.
                    face_equations_valid = false;
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
            }
        }
        if !face_equations_valid {
            for (carrier_index, roots) in face_roots.iter_mut().enumerate() {
                for (split_index, roots) in roots.iter_mut().enumerate() {
                    let edge = edge_index(carrier_index, split_index);
                    *roots = [left_face(edge), right_face(edge)];
                }
            }
            for adjacency in &mut face_adjacency {
                adjacency.clear();
            }
        }
        let mut face_windings = vec![None; edge_count.saturating_mul(2)];

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
                        BezierSplitFragment2::AlgebraicEndpointImages { .. }
                        | BezierSplitFragment2::AnalyticParallel(_)
                        | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        | BezierSplitFragment2::SelectedFiber(_) => false,
                    }
                {
                    continue;
                }
                match self.regularized_fragment_geometric_decision(carrier_index, &split.fragment) {
                    Ok(decision) => {
                        actions[carrier_index][split_index] = Some(decision.action);
                        let [left, right] = decision.side_windings;
                        let roots = face_roots[carrier_index][split_index];
                        propagate_regularized_face_windings(
                            &mut face_windings,
                            &face_adjacency,
                            &winding_jumps,
                            [(roots[0], left), (roots[1], right)],
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
                            face_windings[left].is_some() && face_windings[right].is_some()
                        }
                    })
                });
        if !exact_local_seeds_cover_arrangement {
            let mut retained_bounds = Vec::with_capacity(self.data.carriers.len());
            for carrier in &self.data.carriers {
                let cached = carrier
                    .bounds
                    .get_or_init(|| carrier.geometry.certified_outer_bounds(&self.data.policy));
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
                            } else if let RegionCarrierGeometry::AlgebraicChord(chord) =
                                &carrier.geometry
                            {
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
                        | BezierSplitFragment2::AlgebraicEndpointImages { .. }
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
                            RationalBezierIntersectionPointEvidence2::Exact(outside),
                            RationalBezierIntersectionPointEvidence2::Exact(target.clone()),
                            &self.data.policy,
                        )
                        .map_err(|cause| self.invalid(0, cause))?
                        {
                            Classification::Decided(probe) => probe,
                            Classification::Uncertain(_) => continue,
                        };
                        let evidence = match self.intersect_algebraic_probe_boundary(probe) {
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
                                let after = if let RegionCarrierGeometry::AlgebraicChord(chord) =
                                    &carrier.geometry
                                {
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
                            propagate_regularized_face_windings(
                                &mut face_windings,
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
                                         face_windings: &[Option<Vec<i32>>]|
         -> ExactCurveResult<()> {
            for (derived_carrier_index, roots) in face_roots.iter().enumerate() {
                for (derived_split_index, &[left_face, right_face]) in roots.iter().enumerate() {
                    let derived_edge = edge_index(derived_carrier_index, derived_split_index);
                    if edge_overlapped[derived_carrier_index][derived_split_index]
                        && (!edge_overlap_grouped[derived_edge]
                            || !edge_owns_overlap[derived_carrier_index][derived_split_index])
                    {
                        continue;
                    }
                    let (Some(left), Some(right)) = (
                        face_windings[left_face].as_ref(),
                        face_windings[right_face].as_ref(),
                    ) else {
                        continue;
                    };
                    let left = self
                        .data
                        .first
                        .region_location_from_loop_windings(left)
                        .map_err(|cause| self.invalid(derived_carrier_index, cause))?;
                    let right = self
                        .data
                        .first
                        .region_location_from_loop_windings(right)
                        .map_err(|cause| self.invalid(derived_carrier_index, cause))?;
                    let derived = action_from_result_sides(
                        left == RegionPointLocation::Inside,
                        right == RegionPointLocation::Inside,
                    );
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
        update_actions_from_faces(&mut actions, &face_windings)?;

        let propagate_authored_actions =
            |actions: &mut Vec<Vec<Option<RegionFragmentAction>>>| -> ExactCurveResult<()> {
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
                                    CurveError::Topology(
                                        "a degree-two authored continuation changed regularized faces"
                                            .into(),
                                    ),
                                ));
                            }
                            (Some(action), None) => {
                                actions[outgoing.0][outgoing.1] = Some(action);
                                changed = true;
                            }
                            (None, Some(action)) => {
                                actions[incoming.0][incoming.1] = Some(action);
                                changed = true;
                            }
                            (Some(_), Some(_)) | (None, None) => {}
                        }
                    }
                }
                Ok(())
            };

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
                        | BezierSplitFragment2::AlgebraicEndpointImages { .. } => 3,
                        BezierSplitFragment2::AnalyticParallel(_)
                        | BezierSplitFragment2::SelectedFiber(_) => 4,
                    };
                    (rank, carrier_index, split_index)
                })
            })
            .collect::<Vec<_>>();
        work.sort_by_key(|&(rank, _, _)| rank);
        for (_, carrier_index, split_index) in work {
            if actions[carrier_index][split_index].is_some() {
                continue;
            }
            let split = &topology.split_fragments[carrier_index][split_index];
            match self.regularized_fragment_geometric_decision(carrier_index, &split.fragment) {
                Ok(decision) => {
                    actions[carrier_index][split_index] = Some(decision.action);
                    let [left, right] = decision.side_windings;
                    let roots = face_roots[carrier_index][split_index];
                    propagate_regularized_face_windings(
                        &mut face_windings,
                        &face_adjacency,
                        &winding_jumps,
                        [(roots[0], left), (roots[1], right)],
                    )
                    .map_err(|cause| self.invalid(carrier_index, cause))?;
                }
                Err(error @ ExactCurveError::Blocked(_)) => {
                    blockers[carrier_index][split_index] = Some(error);
                }
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            }

            update_actions_from_faces(&mut actions, &face_windings)?;
            propagate_authored_actions(&mut actions)?;
            if actions.iter().flatten().all(Option::is_some) {
                break;
            }
        }

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
                    || !carrier
                        .geometry
                        .has_certified_injective_image(&self.data.policy)
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
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        if let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment {
            return self.regularized_algebraic_cusp_fragment_decision(carrier_index, fragment);
        }
        if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
            return self.regularized_algebraic_chord_fragment_decision(carrier_index, chord);
        }
        let carrier = &self.data.carriers[carrier_index];
        let max_representatives = match &carrier.geometry {
            RegionCarrierGeometry::Bezier(
                BezierSubcurve2::Quadratic(_) | BezierSubcurve2::RationalQuadratic(_),
            ) => 4,
            RegionCarrierGeometry::Bezier(BezierSubcurve2::Cubic(_)) => 6,
            RegionCarrierGeometry::Bezier(BezierSubcurve2::Rational(curve)) => {
                curve.degree().saturating_mul(2).max(2)
            }
            RegionCarrierGeometry::AnalyticParallel(_) => 4,
            RegionCarrierGeometry::AlgebraicChord(_) => 2,
            RegionCarrierGeometry::AlgebraicCuspSemicircle(_) => 4,
        };
        let fragment_parameter_range = fragment.curve_region_parameter_range();
        let (start, end) = (
            fragment_parameter_range.start(),
            fragment_parameter_range.end(),
        );
        let mut upper = end.clone();
        let mut last_reason = UncertaintyReason::Boundary;
        let local_circular_curve = match fragment {
            BezierSplitFragment2::Materialized { curve, .. }
                if retained_circular_support(curve).is_some() =>
            {
                Some(curve)
            }
            BezierSplitFragment2::Materialized { .. }
            | BezierSplitFragment2::AlgebraicEndpointImages { .. }
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
            BezierSplitFragment2::SelectedFiber(_) => None,
        };
        // Retained circular-conic provenance is a construction certificate for
        // a proper rational parametrization of a nondegenerate minor arc.
        let retained_regular_circle = matches!(
            &carrier.geometry,
            RegionCarrierGeometry::Bezier(curve) if retained_circular_support(curve).is_some()
        );
        // General rational conversion is only a fallback certificate source.
        // Avoid rebuilding a circular quadratic whose native provenance has
        // already supplied everything this classifier needs.
        let rational_geometry = if retained_regular_circle {
            None
        } else {
            match &carrier.geometry {
                RegionCarrierGeometry::Bezier(curve) => {
                    RationalBezier2::try_from_subcurve(curve).ok()
                }
                RegionCarrierGeometry::AnalyticParallel(_)
                | RegionCarrierGeometry::AlgebraicChord(_)
                | RegionCarrierGeometry::AlgebraicCuspSemicircle(_) => None,
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
        let source_endpoint_witness = if local_circular_curve.is_none()
            && retained_regular_circle
            && carrier.start == *start
            && start.as_exact().is_some_and(|boundary| {
                end.as_bezier_parameter()
                    .is_some_and(|end| isolator_touches(end, boundary, true))
            }) {
            start.as_exact().cloned()
        } else if local_circular_curve.is_none()
            && retained_regular_circle
            && carrier.end == *end
            && end.as_exact().is_some_and(|boundary| {
                start
                    .as_bezier_parameter()
                    .is_some_and(|start| isolator_touches(start, boundary, false))
            })
        {
            end.as_exact().cloned()
        } else {
            None
        };
        let mut source_endpoint_witness_attempted = false;
        for _ in 0..max_representatives {
            let (parameter, representative, derivative, derivative_follows_boundary) =
                if let Some(curve) = local_circular_curve {
                    let half = (crate::Real::one() / crate::Real::from(2_u8))
                        .map_err(|cause| self.invalid(carrier_index, cause.into()))?;
                    let representative = match curve.point_at(&half, &self.data.policy) {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            break;
                        }
                    };
                    let derivative_curve = RationalBezier2::try_from_subcurve(curve)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    let derivative =
                        match derivative_curve.derivative_at_classified(&half, &self.data.policy) {
                            Classification::Decided(derivative) => derivative,
                            Classification::Uncertain(reason) => {
                                last_reason = reason;
                                break;
                            }
                        };
                    (None, representative, derivative, true)
                } else {
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
                                .strict_rational_between_ordered(&upper, &self.data.policy)
                                .map_err(|cause| self.invalid(carrier_index, cause))?
                            {
                                Classification::Decided(parameter) => parameter,
                                Classification::Uncertain(reason) => {
                                    last_reason = reason;
                                    break;
                                }
                            };
                            upper = CurveRegionParameter2::from_bezier(BezierParameter2::Exact(
                                parameter.clone(),
                            ));
                            parameter
                        }
                    };
                    let retained_endpoint = source_endpoint_witness
                        .as_ref()
                        .filter(|endpoint| *endpoint == &parameter)
                        .and_then(|endpoint| match &carrier.geometry {
                            RegionCarrierGeometry::Bezier(curve)
                                if endpoint == &crate::Real::zero() =>
                            {
                                Some(curve.endpoint_refs().0.clone())
                            }
                            RegionCarrierGeometry::Bezier(curve)
                                if endpoint == &crate::Real::one() =>
                            {
                                Some(curve.endpoint_refs().1.clone())
                            }
                            RegionCarrierGeometry::Bezier(_)
                            | RegionCarrierGeometry::AnalyticParallel(_)
                            | RegionCarrierGeometry::AlgebraicChord(_)
                            | RegionCarrierGeometry::AlgebraicCuspSemicircle(_) => None,
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
                    (Some(parameter), representative, derivative, false)
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
                    match rational_geometry.as_ref().zip(parameter.as_ref()).map(
                        |(curve, parameter)| {
                            curve.derivative_is_certified_nonzero_at(parameter, &self.data.policy)
                        },
                    ) {
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
            if carrier.reversed && !derivative_follows_boundary {
                tangent_x = -tangent_x;
                tangent_y = -tangent_y;
            }
            let source_parameter = parameter.map(|parameter| {
                CurveRegionParameter2::from_bezier(BezierParameter2::Exact(parameter))
            });
            let left = match self.fragment_side_classification(
                carrier_index,
                &representative,
                source_parameter.as_ref(),
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
                source_parameter.as_ref(),
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
        Err(self.blocked(carrier_index, last_reason))
    }

    fn regularized_algebraic_cusp_fragment_decision(
        &self,
        carrier_index: usize,
        fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let start = CurveRegionParameter2::from_algebraic_cusp(fragment.start_parameter().clone());
        let end = CurveRegionParameter2::from_algebraic_cusp(fragment.end_parameter().clone());
        let parameter = match start
            .strict_rational_between_ordered(&end, &self.data.policy)
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
            representative.exact_rational_point(&self.data.policy),
            tangent.exact_rational_vector(&self.data.policy),
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
        let source_parameter = CurveRegionParameter2::from_algebraic_cusp(
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
                let bounds = match carrier
                    .geometry
                    .certified_outer_bounds_refined(refinement_steps, &self.data.policy)
                {
                    Classification::Decided(bounds) => bounds,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        complete = false;
                        break;
                    }
                };
                let bounds = bounds.certified_rational_outer_envelope().unwrap_or(bounds);
                accumulated = Some(match accumulated {
                    None => bounds,
                    Some(ref accumulated) => {
                        match accumulated.union(&bounds, &CurveContext::STRICT) {
                            Classification::Decided(bounds) => bounds,
                            Classification::Uncertain(reason) => {
                                last_reason = reason;
                                complete = false;
                                break;
                            }
                        }
                    }
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
                RationalBezierIntersectionPointEvidence2::Exact(outside),
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
            let probe_end = CurveRegionParameter2::from_algebraic_chord(probe.end_parameter());
            let evidence = match self.intersect_algebraic_probe_boundary(probe) {
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

            let mut crossings = Vec::<(CurveRegionParameter2, usize, bool)>::new();
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
                .data
                .first
                .region_location_from_loop_windings(&left_windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            let right = self
                .data
                .first
                .region_location_from_loop_windings(&right_windings)
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
        if let RationalBezierIntersectionPointEvidence2::Exact(representative) = representative {
            let (tangent_x, tangent_y) = chord
                .exact_line()
                .map(|line| line.delta())
                .or_else(|| chord.certified_unit_tangent())
                .ok_or_else(|| self.blocked(carrier_index, UncertaintyReason::Unsupported))?;
            let parameter = match chord
                .parameter_at_certified_point(
                    RationalBezierIntersectionPointEvidence2::Exact(representative.clone()),
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(Some(parameter)) => {
                    CurveRegionParameter2::from_algebraic_chord(parameter)
                }
                Classification::Decided(None) => {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "an exact chord rejected its exact representative".into(),
                        ),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(self.blocked(carrier_index, reason));
                }
            };
            let left = self.fragment_side_classification(
                carrier_index,
                &representative,
                Some(&parameter),
                &tangent_x,
                &tangent_y,
                true,
            )?;
            let right = self.fragment_side_classification(
                carrier_index,
                &representative,
                Some(&parameter),
                &tangent_x,
                &tangent_y,
                false,
            )?;
            return Ok(RegularizedFragmentDecision::from_classified_sides(
                left, right,
            ));
        }
        let representative = match representative {
            RationalBezierIntersectionPointEvidence2::Algebraic(representative) => representative,
            representative => {
                return self.regularized_retained_chord_fragment_decision_by_probe(
                    carrier_index,
                    representative,
                );
            }
        };
        let [tangent_x, tangent_y] = chord
            .tangent_coordinate_signs(&self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?;
        let left = self.algebraic_fragment_side_classification(
            carrier_index,
            &representative,
            tangent_x,
            tangent_y,
            true,
        )?;
        let right = self.algebraic_fragment_side_classification(
            carrier_index,
            &representative,
            tangent_x,
            tangent_y,
            false,
        )?;
        Ok(RegularizedFragmentDecision::from_classified_sides(
            left, right,
        ))
    }

    /// Seeds both local faces of a retained chord whose representative lives
    /// in more than one selected field.
    ///
    /// A certified exterior rational point has zero winding in every source
    /// loop. Complete carrier-pair replay orders the crossings of the retained
    /// probe ending at the chord representative. The transverse endpoint
    /// contact identifies which source face precedes the endpoint; crossing
    /// the oriented source chord changes only its loop winding by one. No
    /// Cartesian coordinate or finite side displacement is manufactured.
    #[cold]
    fn regularized_retained_chord_fragment_decision_by_probe(
        &self,
        carrier_index: usize,
        representative: RationalBezierIntersectionPointEvidence2,
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
                    "a retained chord regularization probe references a missing source loop".into(),
                ),
            ));
        }
        let candidate_count = self.data.carriers.len().saturating_mul(2).saturating_add(5);
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
            let Some(outside) = retained_probe_exterior_candidate(&outer_bounds, candidate_index)
            else {
                continue;
            };
            let probe = match crate::BezierAlgebraicChord2::try_new(
                RationalBezierIntersectionPointEvidence2::Exact(outside),
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
            };
            let probe_end = CurveRegionParameter2::from_algebraic_chord(probe.end_parameter());
            let evidence = match self.intersect_algebraic_probe_boundary(probe) {
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

            for contact in evidence.contacts() {
                match contact
                    .first_parameter()
                    .cmp_by_refinement(&probe_end, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?
                {
                    Classification::Decided(Ordering::Less | Ordering::Equal) => {}
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
                Vec::<(&CurveRegionParameter2, &CurveRegionParameter2, usize, bool)>::with_capacity(
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
                match group[0]
                    .0
                    .cmp_by_refinement(&probe_end, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?
                {
                    Classification::Decided(Ordering::Equal) => {
                        let mut source_cross = None;
                        for &(_, _, boundary_index, cross_is_positive) in group {
                            if boundary_index != carrier_index || source_cross.is_some() {
                                last_reason = UncertaintyReason::Boundary;
                                continue 'candidate;
                            }
                            source_cross = Some(cross_is_positive);
                        }
                        let Some(source_cross_is_positive) = source_cross else {
                            last_reason = UncertaintyReason::Predicate;
                            continue 'candidate;
                        };
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
                            .data
                            .first
                            .region_location_from_loop_windings(&left_windings)
                            .map_err(|cause| self.invalid(carrier_index, cause))?;
                        let right = self
                            .data
                            .first
                            .region_location_from_loop_windings(&right_windings)
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
                    Classification::Decided(Ordering::Less) => {}
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
                }

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
                    let Some(winding) = windings.get_mut(loop_index) else {
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
                group_start = group_end;
            }
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
                .data
                .first
                .classify_algebraic_point_from_boundary_side_ray_with_windings(
                    representative,
                    Real::from(direction_x),
                    Real::from(direction_y),
                    carrier.loop_index,
                    carrier.fragment_index,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            match classification {
                Classification::Decided(classification) => return Ok(classification),
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
        source_parameter: Option<&CurveRegionParameter2>,
        tangent_x: &crate::Real,
        tangent_y: &crate::Real,
        left: bool,
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
            match self
                .data
                .first
                .classify_point_from_boundary_side_ray_with_windings(
                    representative,
                    direction_x,
                    direction_y,
                    true,
                    if left {
                        BezierLineCrossingDirection::PositiveToNegative
                    } else {
                        BezierLineCrossingDirection::NegativeToPositive
                    },
                    carrier.loop_index,
                    carrier.fragment_index,
                    source_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(classification) => return Ok(classification),
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                }
            }
        }
        Err(self.blocked(carrier_index, last_reason))
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
        // XOR is union with the intersection's filled side removed. Retain the
        // two exact boundary sets directly when a second Boolean traversal
        // cannot materialize that difference.
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
        CurveRegion2::new(union_loops)
            .and_then(|region| region.with_certified_filled_side_is_left(filled_sides))
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Boolean,
                    CurveFamily2::RationalBezier,
                    cause,
                )
            })
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
        let mut region =
            match CurveRegion2::from_certified_retained_arrangement_traversal(&graph, &traversal) {
                Classification::Decided(region) => region,
                Classification::Uncertain(reason) => return Err(self.blocked(0, reason)),
            };
        region = region
            .with_certified_regularized_filled_left_topology()
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
                    RegionCarrierGeometry::Bezier(curve) => subcurve_is_strict_line_image(curve),
                    RegionCarrierGeometry::AlgebraicChord(chord) => chord.exact_line().is_some(),
                    RegionCarrierGeometry::AnalyticParallel(_)
                    | RegionCarrierGeometry::AlgebraicCuspSemicircle(_) => false,
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
            .and_then(CurveRegion2::with_certified_regularized_filled_left_topology)
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
        CurveRegion2::from_certified_oriented_line_contours(material, holes)
            .map(Some)
            .map_err(|cause| self.invalid(0, cause))
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
                // A symmetric rational witness can land on a tangent or
                // shared-boundary event even though the open arrangement
                // fragment lies in one face. Complete pair replay guarantees
                // that its face cannot change between split events, so probe
                // exact rational witnesses on both sides before propagating a
                // boundary ambiguity.
                let Some((start, end)) = fragment_range(fragment) else {
                    return Err(self.blocked(carrier_index, UncertaintyReason::Unsupported));
                };
                let middle = BezierParameter2::Exact(parameter);
                let mut decided = None;
                for (left, right) in [(start, &middle), (&middle, end)] {
                    let witness = match left
                        .strict_rational_between_ordered(right, &self.data.policy)
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
                RationalBezierIntersectionPointEvidence2::Exact(point) => Some(
                    other_region
                        .classify_point_raw(point, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?,
                ),
                RationalBezierIntersectionPointEvidence2::Algebraic(point) => Some(
                    other_region
                        .classify_algebraic_point_raw(point, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?,
                ),
                RationalBezierIntersectionPointEvidence2::AlgebraicChordPair(_)
                | RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_)
                | RationalBezierIntersectionPointEvidence2::AlgebraicCuspChordDerived(_)
                | RationalBezierIntersectionPointEvidence2::AlgebraicChordParallel(_)
                | RationalBezierIntersectionPointEvidence2::AnalyticParallel(_)
                | RationalBezierIntersectionPointEvidence2::Similarity(_) => None,
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
                    RationalBezierIntersectionPointEvidence2::AlgebraicChordPair(point) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(point) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    RationalBezierIntersectionPointEvidence2::AlgebraicCuspChordDerived(point) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    RationalBezierIntersectionPointEvidence2::AlgebraicChordParallel(point) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    RationalBezierIntersectionPointEvidence2::AnalyticParallel(point) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    RationalBezierIntersectionPointEvidence2::Similarity(point) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    RationalBezierIntersectionPointEvidence2::Exact(_)
                    | RationalBezierIntersectionPointEvidence2::Algebraic(_) => unreachable!(),
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
                            boundary.geometry.certified_outer_bounds(&self.data.policy)
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
        point: RationalBezierIntersectionPointEvidence2,
        boundary_region: &CurveRegion2,
        boundary_operand: CurveRegionBooleanOperand2,
    ) -> ExactCurveResult<Classification<RegionPointLocation>> {
        let direct = match &point {
            RationalBezierIntersectionPointEvidence2::Exact(point) => Some(
                boundary_region
                    .classify_point_raw(point, &self.data.policy)
                    .map_err(|cause| self.invalid(owner_carrier_index, cause))?,
            ),
            RationalBezierIntersectionPointEvidence2::Algebraic(point) => Some(
                boundary_region
                    .classify_algebraic_point_off_boundary_raw(point, &self.data.policy)
                    .map_err(|cause| self.invalid(owner_carrier_index, cause))?,
            ),
            RationalBezierIntersectionPointEvidence2::AlgebraicChordPair(_)
            | RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_)
            | RationalBezierIntersectionPointEvidence2::AlgebraicCuspChordDerived(_)
            | RationalBezierIntersectionPointEvidence2::AlgebraicChordParallel(_)
            | RationalBezierIntersectionPointEvidence2::AnalyticParallel(_)
            | RationalBezierIntersectionPointEvidence2::Similarity(_) => None,
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
        point: RationalBezierIntersectionPointEvidence2,
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
        point: RationalBezierIntersectionPointEvidence2,
        boundary_operand: CurveRegionBooleanOperand2,
        classification_kind: RetainedPointProbeClassification,
    ) -> ExactCurveResult<Classification<RegionPointLocation>> {
        let boundary_region = match boundary_operand {
            CurveRegionBooleanOperand2::First => self.data.first,
            CurveRegionBooleanOperand2::Second => self.data.second,
        };
        let boundary_carriers = self
            .data
            .carriers
            .iter()
            .filter(|carrier| carrier.operand == boundary_operand)
            .cloned()
            .collect::<Vec<_>>();
        if boundary_carriers.is_empty() {
            return Ok(Classification::Decided(RegionPointLocation::Outside));
        }

        let mut last_reason = UncertaintyReason::Unsupported;
        let outer_bounds = match retained_probe_outer_bounds(&boundary_carriers, &self.data.policy)
        {
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
                RationalBezierIntersectionPointEvidence2::Exact(outside),
                point.clone(),
                &self.data.policy,
            )
            .map_err(|cause| self.invalid(owner_carrier_index, cause))?
            {
                Classification::Decided(probe) => probe,
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                    continue;
                }
            };
            let probe_end = CurveRegionParameter2::from_algebraic_chord(probe.end_parameter());
            let evidence = match self.intersect_algebraic_probe_carriers(
                probe,
                boundary_region,
                &boundary_carriers,
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

            let mut crossings = Vec::<(&CurveRegionParameter2, bool, bool)>::with_capacity(
                evidence.contacts().len(),
            );
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
                    boundary.filled_side_is_left,
                ));
            }
            if ambiguous {
                continue;
            }
            match classification_kind {
                RetainedPointProbeClassification::FilledRegion => {
                    let mut last_contact = None::<(&CurveRegionParameter2, RegionPointLocation)>;
                    for (parameter, cross_is_positive, filled_side_is_left) in crossings {
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
            let coarse_bounds_are_separated = || {
                let Classification::Decided(carrier_bounds) = carrier
                    .bounds
                    .get_or_init(|| carrier.geometry.certified_outer_bounds(&self.data.policy))
                else {
                    return false;
                };
                let mut other_bounds = None::<Aabb2>;
                for other in &self.data.carriers {
                    if other.operand == carrier.operand {
                        continue;
                    }
                    let Classification::Decided(bounds) = other
                        .bounds
                        .get_or_init(|| other.geometry.certified_outer_bounds(&self.data.policy))
                    else {
                        return false;
                    };
                    other_bounds = Some(match other_bounds {
                        None => bounds.clone(),
                        Some(accumulated) => match accumulated.union(bounds, &self.data.policy) {
                            Classification::Decided(bounds) => bounds,
                            Classification::Uncertain(_) => return false,
                        },
                    });
                }
                other_bounds.is_none_or(|other_bounds| {
                    carrier_bounds.overlaps(&other_bounds, &self.data.policy)
                        == Classification::Decided(false)
                })
            };
            coarse_bounds_are_separated()
                || [0, 2, 4, 8, 16, 32, 64, 128, 256, 512]
                    .into_iter()
                    .any(|refinement_steps| {
                        let Classification::Decided(carrier_bounds) = carrier
                            .geometry
                            .certified_outer_bounds_refined(refinement_steps, &self.data.policy)
                        else {
                            return false;
                        };
                        let mut other_bounds = None::<Aabb2>;
                        for other in &self.data.carriers {
                            if other.operand == carrier.operand {
                                continue;
                            }
                            let other_result = other.geometry.certified_outer_bounds_refined(
                                refinement_steps,
                                &self.data.policy,
                            );
                            let Classification::Decided(bounds) = other_result else {
                                return false;
                            };
                            other_bounds = Some(match other_bounds {
                                None => bounds,
                                Some(accumulated) => {
                                    let Classification::Decided(bounds) =
                                        accumulated.union(&bounds, &self.data.policy)
                                    else {
                                        return false;
                                    };
                                    bounds
                                }
                            });
                        }
                        other_bounds.is_none_or(|other_bounds| {
                            carrier_bounds.overlaps(&other_bounds, &self.data.policy)
                                == Classification::Decided(false)
                        })
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
            .strict_rational_between_ordered(end, &self.data.policy)
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
        let same_source_direction = overlap.orientation == RationalBezierOverlapOrientation2::Same;
        let same_traversal = same_source_direction == (first.reversed == second.reversed);
        let second_left_in_first_direction = if same_traversal {
            second.filled_side_is_left
        } else {
            !second.filled_side_is_left
        };
        let left = operation.apply(first.filled_side_is_left, second_left_in_first_direction);
        let right = operation.apply(!first.filled_side_is_left, !second_left_in_first_direction);
        Ok(action_from_result_sides(left, right))
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
    if carrier_bounds_decided_disjoint(first_carrier, second_carrier, policy) {
        return Ok(None);
    }
    let context = match (&first_carrier.geometry, &second_carrier.geometry) {
        (
            RegionCarrierGeometry::AlgebraicCuspSemicircle(_),
            RegionCarrierGeometry::AlgebraicChord(_),
        ) => RegionCarrierPairContext::CuspChord {
            cusp_is_first: true,
        },
        (
            RegionCarrierGeometry::AlgebraicChord(_),
            RegionCarrierGeometry::AlgebraicCuspSemicircle(_),
        ) => RegionCarrierPairContext::CuspChord {
            cusp_is_first: false,
        },
        (RegionCarrierGeometry::AlgebraicChord(_), _)
        | (_, RegionCarrierGeometry::AlgebraicChord(_)) => {
            RegionCarrierPairContext::AlgebraicChordPair
        }
        (RegionCarrierGeometry::Bezier(_), RegionCarrierGeometry::Bezier(_)) => {
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
            RegionCarrierPairContext::Bezier(context)
        }
        (RegionCarrierGeometry::AnalyticParallel(_), RegionCarrierGeometry::Bezier(_)) => {
            RegionCarrierPairContext::ParallelRational {
                parallel_is_first: true,
            }
        }
        (RegionCarrierGeometry::Bezier(_), RegionCarrierGeometry::AnalyticParallel(_)) => {
            RegionCarrierPairContext::ParallelRational {
                parallel_is_first: false,
            }
        }
        (
            RegionCarrierGeometry::AnalyticParallel(first),
            RegionCarrierGeometry::AnalyticParallel(second),
        ) => {
            if first == second {
                RegionCarrierPairContext::ParallelSameImage
            } else {
                RegionCarrierPairContext::ParallelPair
            }
        }
        (RegionCarrierGeometry::AlgebraicCuspSemicircle(_), RegionCarrierGeometry::Bezier(_)) => {
            RegionCarrierPairContext::CuspRational {
                cusp_is_first: true,
            }
        }
        (RegionCarrierGeometry::Bezier(_), RegionCarrierGeometry::AlgebraicCuspSemicircle(_)) => {
            RegionCarrierPairContext::CuspRational {
                cusp_is_first: false,
            }
        }
        (
            RegionCarrierGeometry::AlgebraicCuspSemicircle(_),
            RegionCarrierGeometry::AnalyticParallel(_),
        ) => RegionCarrierPairContext::CuspParallel {
            cusp_is_first: true,
        },
        (
            RegionCarrierGeometry::AnalyticParallel(_),
            RegionCarrierGeometry::AlgebraicCuspSemicircle(_),
        ) => RegionCarrierPairContext::CuspParallel {
            cusp_is_first: false,
        },
        (
            RegionCarrierGeometry::AlgebraicCuspSemicircle(_),
            RegionCarrierGeometry::AlgebraicCuspSemicircle(_),
        ) => RegionCarrierPairContext::CuspPair,
    };
    Ok(Some(RegionCarrierPair {
        first_carrier_index,
        second_carrier_index,
        context,
    }))
}

fn build_parallel_self_intersection_caches(
    carriers: &[RegionCarrier],
) -> Vec<ParallelSelfIntersectionCache> {
    let mut caches = Vec::<ParallelSelfIntersectionCache>::new();
    for carrier in carriers {
        let RegionCarrierGeometry::AnalyticParallel(parallel) = &carrier.geometry else {
            continue;
        };
        if caches.iter().any(|cache| cache.parallel == *parallel) {
            continue;
        }
        caches.push(ParallelSelfIntersectionCache {
            parallel: parallel.clone(),
            result: OnceLock::new(),
        });
    }
    caches
}

fn build_bezier_self_intersection_caches(
    carriers: &[RegionCarrier],
    pairs: &[RegionCarrierPair],
) -> Vec<BezierSelfIntersectionCache> {
    let mut caches = Vec::<BezierSelfIntersectionCache>::new();
    for pair in pairs {
        if !matches!(pair.context, RegionCarrierPairContext::BezierSelf) {
            continue;
        }
        let curve = carriers[pair.first_carrier_index].geometry.bezier();
        if caches.iter().any(|cache| cache.curve == *curve) {
            continue;
        }
        caches.push(BezierSelfIntersectionCache {
            curve: curve.clone(),
            result: OnceLock::new(),
        });
    }
    caches
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
            RationalBezierIntersectionPointEvidence2::Exact(line.start().clone()),
            RationalBezierIntersectionPointEvidence2::Exact(line.end().clone()),
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
        let first_bounds = first
            .bounds
            .get_or_init(|| first.geometry.certified_outer_bounds(policy));
        let second_bounds = second
            .bounds
            .get_or_init(|| second.geometry.certified_outer_bounds(policy));
        match (first_bounds, second_bounds) {
            (Classification::Decided(first), Classification::Decided(second)) => matches!(
                first.overlaps(second, policy),
                Classification::Decided(false)
            ),
            (Classification::Decided(_) | Classification::Uncertain(_), _) => false,
        }
    })
}

fn carrier_bounds_refined(
    carrier: &RegionCarrier,
    refinement_steps: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<Aabb2>> {
    match &carrier.geometry {
        RegionCarrierGeometry::AlgebraicChord(chord) => {
            chord.conservative_bounds_refined(refinement_steps, policy)
        }
        RegionCarrierGeometry::AlgebraicCuspSemicircle(fragment) => fragment
            .semicircle()
            .conservative_bounds_refined(refinement_steps, policy),
        RegionCarrierGeometry::Bezier(curve) => {
            let (Some(start), Some(end)) = (
                carrier.start.as_bezier_parameter(),
                carrier.end.as_bezier_parameter(),
            ) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let start = start
                .clone()
                .refined_isolating_interval(refinement_steps, policy);
            let end = end
                .clone()
                .refined_isolating_interval(refinement_steps, policy);
            let start = match start.known_interval(policy)? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let end = match end.known_interval(policy)? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let source = RationalBezier2::try_from_subcurve(curve)?;
            let subcurve = match source.subcurve_between_exact(start.start(), end.end(), policy)? {
                Classification::Decided(subcurve) => subcurve,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(subcurve.certified_bounds_classified(policy))
        }
        RegionCarrierGeometry::AnalyticParallel(_) => {
            Ok(carrier.geometry.certified_outer_bounds(policy))
        }
    }
}

fn carrier_refined_bounds_decided_disjoint(
    first: &RegionCarrier,
    second: &RegionCarrier,
    policy: &CurveContext,
) -> CurveResult<bool> {
    // This is a broad-phase rejection proof, not a terminal predicate.
    // Unresolved boxes proceed to the authoritative exact pair kernel.
    policy.strict_predicate_pass(|| {
        for refinement_steps in [0, 2] {
            let (Classification::Decided(first_bounds), Classification::Decided(second_bounds)) = (
                carrier_bounds_refined(first, refinement_steps, policy)?,
                carrier_bounds_refined(second, refinement_steps, policy)?,
            ) else {
                continue;
            };
            if first_bounds.overlaps(&second_bounds, policy) == Classification::Decided(false) {
                return Ok(true);
            }
        }
        Ok(false)
    })
}

fn subcurve_certified_outer_bounds(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let bounds = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.control_hull_box(policy),
        BezierSubcurve2::Cubic(curve) => curve.control_hull_box(policy),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(policy),
    };
    if matches!(bounds, Classification::Decided(_)) {
        return bounds;
    }
    let Some((_, circle)) = retained_circular_support(curve) else {
        return bounds;
    };
    // Mixed-weight major-circle charts are finite even though their rational
    // control hull is not convex. Retained circular provenance certifies the
    // complete image lies in this exact full-circle envelope, which is a
    // conservative broad-phase fallback when quotient-extremum isolation did
    // not produce a tighter box.
    let radius = match circle.radius_squared.clone().sqrt() {
        Ok(radius) => radius,
        Err(_) => return bounds,
    };
    Classification::Decided(Aabb2::new_unchecked(
        crate::Point2::new(circle.center.x() - &radius, circle.center.y() - &radius),
        crate::Point2::new(circle.center.x() + &radius, circle.center.y() + &radius),
    ))
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

fn build_region_carrier(
    fragment: &BezierSplitFragment2,
    operand: CurveRegionBooleanOperand2,
    loop_index: usize,
    fragment_index: usize,
    filled_side_is_left: bool,
    policy: &CurveContext,
) -> ExactCurveResult<RegionCarrier> {
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
    let (mut geometry, mut start, mut end, mut reversed) = match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            if let Some(chord) = retained_axis_aligned_line_chord(curve, policy) {
                let start = chord.start_parameter();
                let end = chord.end_parameter();
                (
                    RegionCarrierGeometry::AlgebraicChord(chord),
                    CurveRegionParameter2::from_algebraic_chord(start),
                    CurveRegionParameter2::from_algebraic_chord(end),
                    false,
                )
            } else {
                (
                    RegionCarrierGeometry::Bezier(curve.clone()),
                    CurveRegionParameter2::from_bezier(
                        BezierParameter2::Exact(crate::Real::zero()),
                    ),
                    CurveRegionParameter2::from_bezier(BezierParameter2::Exact(crate::Real::one())),
                    false,
                )
            }
        }
        BezierSplitFragment2::AlgebraicEndpointImages {
            reversed,
            start,
            end,
            source_curve: curve,
            ..
        } => (
            RegionCarrierGeometry::Bezier(curve.clone()),
            CurveRegionParameter2::from_bezier(start.clone()),
            CurveRegionParameter2::from_bezier(end.clone()),
            *reversed,
        ),
        BezierSplitFragment2::AnalyticParallel(fragment) => (
            RegionCarrierGeometry::AnalyticParallel(fragment.parallel().clone()),
            CurveRegionParameter2::from_bezier(fragment.range().start().clone()),
            CurveRegionParameter2::from_bezier(fragment.range().end().clone()),
            fragment.is_reversed(),
        ),
        BezierSplitFragment2::AlgebraicChord(chord) => (
            RegionCarrierGeometry::AlgebraicChord(chord.clone()),
            CurveRegionParameter2::from_algebraic_chord(chord.start_parameter()),
            CurveRegionParameter2::from_algebraic_chord(chord.end_parameter()),
            false,
        ),
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => (
            RegionCarrierGeometry::AlgebraicCuspSemicircle(fragment.clone()),
            CurveRegionParameter2::from_algebraic_cusp(fragment.start_parameter().clone()),
            CurveRegionParameter2::from_algebraic_cusp(fragment.end_parameter().clone()),
            fragment.is_reversed(),
        ),
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let geometry = match fragment.source() {
                BezierSelectedFiberSource2::Rational(curve) => {
                    RegionCarrierGeometry::Bezier(BezierSubcurve2::Rational(curve.clone()))
                }
                BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
                    RegionCarrierGeometry::AnalyticParallel(parallel.clone())
                }
            };
            (
                geometry,
                fragment.range().start().clone(),
                fragment.range().end().clone(),
                fragment.is_reversed(),
            )
        }
    };
    if matches!(
        fragment,
        BezierSplitFragment2::AlgebraicEndpointImages { .. }
    ) && let Ok(Classification::Decided(line)) =
        crate::bezier_region::retained_line_fragment_segment(fragment, policy)
    {
        geometry = RegionCarrierGeometry::Bezier(BezierSubcurve2::Quadratic(
            QuadraticBezier2::from_line_segment(line),
        ));
        start = CurveRegionParameter2::from_bezier(BezierParameter2::Exact(crate::Real::zero()));
        end = CurveRegionParameter2::from_bezier(BezierParameter2::Exact(crate::Real::one()));
        reversed = false;
    }
    let family = geometry.family();
    Ok(RegionCarrier {
        operand,
        loop_index,
        fragment_index,
        family,
        geometry,
        start,
        end,
        reversed,
        filled_side_is_left,
        selected_fiber_endpoint_points,
        image_is_injective: OnceLock::new(),
        bounds: OnceLock::new(),
    })
}

fn split_carrier(
    carrier: &RegionCarrier,
    events: &[CarrierEvent],
    contact_points: &[ContactVertex],
    exact_contact_point_index_by_vertex: &[usize],
    policy: &CurveContext,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
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
    // carrier. Only generic whole-curve materialization needs a second range
    // clip below.
    if carrier.selected_fiber_endpoint_points.is_some()
        || events
            .iter()
            .any(|event| event.parameter.is_retained_scalar())
    {
        return split_selected_fiber_carrier(carrier, events, contact_points, policy);
    }
    if let RegionCarrierGeometry::AnalyticParallel(parallel) = &carrier.geometry {
        return split_analytic_carrier(carrier, parallel, events, max_refinement_steps, policy);
    }
    if let RegionCarrierGeometry::AlgebraicChord(chord) = &carrier.geometry {
        return split_algebraic_chord_carrier(carrier, chord, events, policy);
    }
    if let RegionCarrierGeometry::AlgebraicCuspSemicircle(fragment) = &carrier.geometry {
        return split_algebraic_cusp_carrier(carrier, fragment, events, contact_points, policy);
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
    let materialization = match carrier
        .geometry
        .bezier()
        .split_at_parameters_refined(&parameters, policy)?
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
        let start_parameter = CurveRegionParameter2::from_bezier(start.clone());
        let end_parameter = CurveRegionParameter2::from_bezier(end.clone());
        if !parameter_range_inside_carrier(&start_parameter, &end_parameter, carrier, policy)? {
            continue;
        }
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
) -> Result<RationalBezierIntersectionPointEvidence2, CurveError> {
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
            exact_contact_point_evidence(curve, &parameter, policy)?.ok_or_else(|| {
                CurveError::Topology(
                    "a selected-fiber rational boundary could not retain its exact point".into(),
                )
            })
        }
        BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
            if let Some(parameter) = parameter.as_exact() {
                return match parallel.point_at(parameter, policy)? {
                    Classification::Decided(point) => {
                        Ok(RationalBezierIntersectionPointEvidence2::Exact(point))
                    }
                    Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                        "selected-fiber analytic endpoint evaluation remained uncertain: {reason:?}"
                    ))),
                };
            }
            Ok(RationalBezierIntersectionPointEvidence2::AnalyticParallel(
                crate::BezierAnalyticParallelPoint2::new(parallel.clone(), parameter, policy),
            ))
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
        RegionCarrierGeometry::Bezier(curve) => {
            BezierSelectedFiberSource2::Rational(RationalBezier2::try_from_subcurve(curve)?)
        }
        RegionCarrierGeometry::AnalyticParallel(parallel) => {
            BezierSelectedFiberSource2::AnalyticParallel(parallel.clone())
        }
        RegionCarrierGeometry::AlgebraicChord(_)
        | RegionCarrierGeometry::AlgebraicCuspSemicircle(_) => {
            return Err(CurveError::Topology(
                "a selected-fiber boundary reached an incompatible carrier".into(),
            ));
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
            fragment: BezierSplitFragment2::SelectedFiber(BezierSelectedFiberFragment2::new(
                source.clone(),
                CurveRegionParameterRange2::new_validated(
                    pair[0].parameter.clone(),
                    pair[1].parameter.clone(),
                ),
                start_point,
                end_point,
            )),
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
        match pair[0]
            .parameter
            .cmp_by_refinement(&pair[1].parameter, policy)?
        {
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
        let Some(start) = pair[0].parameter.as_algebraic_chord() else {
            return Err(CurveError::Topology(
                "non-chord cut reached an algebraic chord carrier".into(),
            ));
        };
        let Some(end) = pair[1].parameter.as_algebraic_chord() else {
            return Err(CurveError::Topology(
                "non-chord cut reached an algebraic chord carrier".into(),
            ));
        };
        let retained =
            crate::BezierAlgebraicChord2::from_ordered_parameter_range(chord, start, end, policy)?;
        output.push(SplitCarrierFragment {
            fragment: BezierSplitFragment2::AlgebraicChord(retained),
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
        let retained = crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
            fragment.semicircle().clone(),
            start.clone(),
            end.clone(),
            false,
            policy,
        )
        .inherit_certified_tangent_endpoints(fragment);
        output.push(SplitCarrierFragment {
            fragment: BezierSplitFragment2::AlgebraicCuspSemicircle(retained),
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
    parallel: &BezierParallel2,
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
                parameter: CurveRegionParameter2::from_bezier(
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
            fragment: BezierSplitFragment2::AnalyticParallel(
                crate::BezierParallelFragment2::from_certified_range(
                    parallel.clone(),
                    BezierParameterRange2::new_validated(start, end),
                    false,
                ),
            ),
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
    let BezierSplitFragment2::AlgebraicEndpointImages { start, end, .. } = fragment else {
        return fragment.clone();
    };
    let RegionCarrierGeometry::Bezier(carrier_curve) = &carrier.geometry else {
        return fragment.clone();
    };
    let Some((implicit_conic, circular_conic)) = retained_circular_support(carrier_curve) else {
        return fragment.clone();
    };
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
    let curve = match curve.retained_quadratic_representative(policy).ok()? {
        Classification::Decided(Some(curve)) => curve,
        Classification::Decided(None) | Classification::Uncertain(_) => return None,
    };
    (curve.retained_implicit_quadratic_conic().is_some()
        && curve.retained_circular_conic().is_some())
    .then_some(curve)
}

fn retained_circular_support(
    curve: &BezierSubcurve2,
) -> Option<(
    &Arc<[crate::Real; 6]>,
    &Arc<crate::rational_bezier::RationalQuadraticCircle2>,
)> {
    let (implicit, circular) = match curve {
        BezierSubcurve2::RationalQuadratic(curve) => (
            curve.retained_implicit_quadratic_conic(),
            curve.retained_circular_conic(),
        ),
        BezierSubcurve2::Rational(curve) => (
            curve.retained_implicit_quadratic_conic(),
            curve.retained_circular_conic(),
        ),
        BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => return None,
    };
    Some((implicit?, circular?))
}

fn adjacent_axis_algebraic_chord_circular_curve_is_endpoint_only(
    chord: &crate::BezierAlgebraicChord2,
    chord_carrier: &RegionCarrier,
    curve: &BezierSubcurve2,
    curve_carrier: &RegionCarrier,
    chord_precedes_curve: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if !curve_carrier.geometry.has_certified_injective_image(policy) {
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
        .and_then(|parameter| parameter.point().as_exact())
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
        && let Some(RationalBezierIntersectionPointEvidence2::Exact(point)) =
            &contact_points.get(contact_index)?.point
    {
        return Some(point.clone());
    }
    let parameter = parameter.as_exact()?;
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
) -> Vec<Option<usize>> {
    let starts_by_vertex = arrangement_starts_by_vertex(graph, None);
    let mut successors = certified_transverse_successors(
        graph,
        directions,
        &topology.transverse_contacts,
        &starts_by_vertex,
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
) -> Vec<Option<usize>> {
    let starts_by_vertex = arrangement_starts_by_vertex(graph, None);
    let transverse_successors = certified_transverse_successors(
        graph,
        directions,
        &topology.transverse_contacts,
        &starts_by_vertex,
        |contact, _| contact.cross_is_positive,
    );
    let mut successors = (0..graph.len())
        .map(|index| {
            face_sector_successors
                .get(index)
                .copied()
                .flatten()
                .or_else(|| transverse_successors.get(index).copied().flatten())
        })
        .collect::<Vec<_>>();
    certify_nontransverse_regularization_authored_continuity(
        &mut successors,
        graph,
        directions,
        topology,
        carriers,
        &starts_by_vertex,
    );
    successors
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
            certified_turn_preference(current, first, second, crossing_is_positive).map(
                |first_before_second| {
                    if first_before_second {
                        first_index
                    } else {
                        second_index
                    }
                },
            )
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
) -> Option<bool> {
    let first_half = certified_turn_half(base, first, crossing_is_positive)?;
    let second_half = certified_turn_half(base, second, crossing_is_positive)?;
    if first_half != second_half {
        return Some(first_half < second_half);
    }
    match certified_direction_cross(first, second, crossing_is_positive)? {
        1 => Some(true),
        -1 => Some(false),
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
            match algebraic_endpoint_tangent_cross_sign(first, second, policy) {
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
) -> Option<&BezierEndpointTangentImage2> {
    fragments.iter().find_map(|split| {
        let BezierSplitFragment2::AlgebraicEndpointImages {
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
                .and_then(|image| image.try_tangent().ok());
        }
        if split.end_topology_vertex == Some(vertex) {
            return if *reversed { start_image } else { end_image }
                .as_ref()
                .and_then(|image| image.try_tangent().ok());
        }
        None
    })
}

#[cfg(test)]
fn push_carrier_event(
    events: &mut Vec<CarrierEvent>,
    parameter: CurveRegionParameter2,
    topology_vertex: Option<usize>,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    push_carrier_event_internal(events, parameter, topology_vertex, carrier, false, policy)
        .map(|_| ())
}

fn push_canonical_carrier_event(
    events: &mut Vec<CarrierEvent>,
    parameter: CurveRegionParameter2,
    topology_vertex: Option<usize>,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegionParameter2> {
    let (_, event_index) =
        push_carrier_event_internal(events, parameter, topology_vertex, carrier, false, policy)?;
    Ok(events[event_index].parameter.clone())
}

fn push_contact_carrier_event(
    events: &mut Vec<CarrierEvent>,
    parameter: CurveRegionParameter2,
    topology_vertex: Option<usize>,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    push_carrier_event_internal(events, parameter, topology_vertex, carrier, true, policy)
        .map(|(deferred, _)| deferred)
}

fn push_carrier_event_internal(
    events: &mut Vec<CarrierEvent>,
    parameter: CurveRegionParameter2,
    topology_vertex: Option<usize>,
    carrier: &RegionCarrier,
    defer_unordered: bool,
    policy: &CurveContext,
) -> ExactCurveResult<(bool, usize)> {
    let mut deferred_ordering = false;
    for (event_index, event) in events.iter_mut().enumerate() {
        let same_topology_vertex =
            topology_vertex.is_some() && event.topology_vertex == topology_vertex;
        match parameter
            .cmp_by_refinement(&event.parameter, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Boolean, carrier.family, cause)
            })? {
            Classification::Decided(Ordering::Equal) => {
                if event.topology_vertex.is_none() {
                    event.topology_vertex = topology_vertex;
                }
                return Ok((deferred_ordering, event_index));
            }
            Classification::Decided(_)
                if same_topology_vertex
                    && carrier_has_certified_injective_image(carrier, policy) =>
            {
                return Ok((deferred_ordering, event_index));
            }
            Classification::Decided(_) => {}
            Classification::Uncertain(_)
                if same_topology_vertex
                    && carrier_has_certified_injective_image(carrier, policy) =>
            {
                return Ok((deferred_ordering, event_index));
            }
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

fn carrier_traversal_start(carrier: &RegionCarrier) -> &CurveRegionParameter2 {
    if carrier.reversed {
        &carrier.end
    } else {
        &carrier.start
    }
}

fn carrier_traversal_end(carrier: &RegionCarrier) -> &CurveRegionParameter2 {
    if carrier.reversed {
        &carrier.start
    } else {
        &carrier.end
    }
}

fn existing_event_vertex(
    events: &[CarrierEvent],
    parameter: &CurveRegionParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<usize>> {
    for event in events {
        match decided_parameter_cmp(parameter, &event.parameter, policy)? {
            Ordering::Equal => return Ok(event.topology_vertex),
            Ordering::Less | Ordering::Greater => {}
        }
    }
    Ok(None)
}

fn existing_event_vertex_if_decided(
    events: &[CarrierEvent],
    parameter: &CurveRegionParameter2,
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

fn carrier_has_certified_injective_image(carrier: &RegionCarrier, policy: &CurveContext) -> bool {
    if let Some(&injective) = carrier.image_is_injective.get() {
        return injective;
    }
    let injective = carrier.geometry.has_certified_injective_image(policy);
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
    parameters: [&CurveRegionParameter2; 2],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    for (existing_slot, existing_carrier) in existing.carrier_indices.iter().copied().enumerate() {
        for (current_slot, current_carrier) in carrier_indices.iter().copied().enumerate() {
            let (
                RegionCarrierGeometry::AnalyticParallel(existing_parallel),
                RegionCarrierGeometry::AnalyticParallel(current_parallel),
            ) = (
                &carriers[existing_carrier].geometry,
                &carriers[current_carrier].geometry,
            )
            else {
                continue;
            };
            if existing_parallel == current_parallel
                && matches!(
                    existing.parameters[existing_slot]
                        .cmp_by_refinement(parameters[current_slot], policy)
                        .map_err(|cause| ExactCurveError::invalid(
                            CurveOperation2::Boolean,
                            carriers[existing_carrier].family,
                            cause,
                        ))?,
                    Classification::Decided(order) if order != Ordering::Equal
                )
                && existing_parallel.regular_fragment_has_certified_injective_axis(policy)
            {
                return Ok(true);
            }
        }
    }
    for existing_carrier in existing.carrier_indices {
        for current_carrier in carrier_indices {
            let existing_bounds = carriers[existing_carrier].bounds.get_or_init(|| {
                carriers[existing_carrier]
                    .geometry
                    .certified_outer_bounds(policy)
            });
            let current_bounds = carriers[current_carrier].bounds.get_or_init(|| {
                carriers[current_carrier]
                    .geometry
                    .certified_outer_bounds(policy)
            });
            if let (
                Classification::Decided(existing_bounds),
                Classification::Decided(current_bounds),
            ) = (existing_bounds, current_bounds)
                && existing_bounds.overlaps(current_bounds, policy)
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
                    existing.parameters[existing_slot]
                        .cmp_by_refinement(parameters[current_slot], policy)
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

fn contacts_decided_same_from_shared_parallel(
    existing: &ContactVertex,
    carrier_indices: [usize; 2],
    parameters: [&CurveRegionParameter2; 2],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    for (existing_slot, existing_carrier) in existing.carrier_indices.iter().copied().enumerate() {
        for (current_slot, current_carrier) in carrier_indices.iter().copied().enumerate() {
            let (
                RegionCarrierGeometry::AnalyticParallel(existing_parallel),
                RegionCarrierGeometry::AnalyticParallel(current_parallel),
            ) = (
                &carriers[existing_carrier].geometry,
                &carriers[current_carrier].geometry,
            )
            else {
                continue;
            };
            if existing_parallel == current_parallel
                && matches!(
                    existing.parameters[existing_slot]
                        .cmp_by_refinement(parameters[current_slot], policy)
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
    parameter: &CurveRegionParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<&'a RationalBezierIntersectionPointEvidence2>> {
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
    parameter: &CurveRegionParameter2,
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
    parameters: [&CurveRegionParameter2; 2],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> Classification<bool> {
    let mut uncertainty = None;
    for (existing_slot, existing_carrier) in existing.carrier_indices.iter().copied().enumerate() {
        let RegionCarrierGeometry::Bezier(existing_subcurve) = &carriers[existing_carrier].geometry
        else {
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
            let RegionCarrierGeometry::Bezier(current_subcurve) =
                &carriers[current_carrier].geometry
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
    let RegionCarrierGeometry::Bezier(curve) = &carrier.geometry else {
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
        let RegionCarrierGeometry::Bezier(curve) = &carriers[carrier_index].geometry else {
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
    parameter: &CurveRegionParameter2,
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

fn cusp_contact_parameter(
    location: BezierAlgebraicCuspSemicircleContactLocation2,
) -> Option<BezierAlgebraicCuspSemicircleParameter2> {
    match location {
        BezierAlgebraicCuspSemicircleContactLocation2::Start => {
            Some(BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()))
        }
        BezierAlgebraicCuspSemicircleContactLocation2::End => {
            Some(BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one()))
        }
        BezierAlgebraicCuspSemicircleContactLocation2::Interior => None,
    }
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

fn selected_fiber_cusp_contacts_result(
    contacts: Vec<BezierAlgebraicCuspSemicircleSelectedFiberContact2>,
    cusp_is_first: bool,
) -> RegionPairResult {
    let contacts = contacts
        .into_iter()
        .map(|contact| {
            let cusp_parameter =
                CurveRegionParameter2::from_algebraic_cusp(contact.cusp_parameter());
            let other_parameter =
                CurveRegionParameter2::from_selected_fiber(contact.other_parameter().clone());
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
        overlaps: Vec::new(),
        blockers: Vec::new(),
    }
}

fn selected_fiber_cusp_overlaps_result(
    overlaps: Vec<BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2>,
    cusp_is_first: bool,
) -> RegionPairResult {
    let overlaps = overlaps
        .into_iter()
        .map(|overlap| {
            let orientation = overlap.orientation();
            let cusp_range = CurveRegionParameterRange2::new_validated(
                CurveRegionParameter2::from_algebraic_cusp(overlap.cusp_start_parameter()),
                CurveRegionParameter2::from_algebraic_cusp(overlap.cusp_end_parameter()),
            );
            let other_range = CurveRegionParameterRange2::new_validated(
                CurveRegionParameter2::from_selected_fiber(overlap.other_start_parameter()),
                CurveRegionParameter2::from_selected_fiber(overlap.other_end_parameter()),
            );
            let (first_range, second_range) = if cusp_is_first {
                (cusp_range, other_range)
            } else {
                (other_range, cusp_range)
            };
            RegionPairOverlap {
                source: Some(RegionPairOverlapSource::AlgebraicCuspSelectedFiberMapped(
                    overlap,
                )),
                first_range,
                second_range,
                orientation,
            }
        })
        .collect();
    RegionPairResult {
        contacts: Vec::new(),
        overlaps,
        blockers: Vec::new(),
    }
}

const fn action_from_result_sides(left: bool, right: bool) -> RegionFragmentAction {
    match (left, right) {
        (true, false) => RegionFragmentAction::Keep,
        (false, true) => RegionFragmentAction::KeepReversed,
        (false, false) | (true, true) => RegionFragmentAction::Discard,
    }
}

const fn carrier_traversal_start_parameter(carrier: &RegionCarrier) -> &CurveRegionParameter2 {
    if carrier.reversed {
        &carrier.end
    } else {
        &carrier.start
    }
}

const fn carrier_traversal_end_parameter(carrier: &RegionCarrier) -> &CurveRegionParameter2 {
    if carrier.reversed {
        &carrier.start
    } else {
        &carrier.end
    }
}

fn exact_carrier_point(
    carrier: &RegionCarrier,
    parameter: &CurveRegionParameter2,
    policy: &CurveContext,
) -> Option<crate::Point2> {
    let parameter = parameter.as_exact()?;
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
    parameter: &CurveRegionParameter2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    Ok(parameter_location_in_carrier(parameter, carrier, policy)?
        != CarrierParameterLocation::Outside)
}

fn parameter_location_in_carrier(
    parameter: &CurveRegionParameter2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<CarrierParameterLocation> {
    if parameter == &carrier.start || parameter == &carrier.end {
        return Ok(CarrierParameterLocation::Endpoint);
    }
    if let (Some(parameter), RegionCarrierGeometry::AlgebraicCuspSemicircle(fragment)) =
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
    if let (Some(parameter), RegionCarrierGeometry::AlgebraicCuspSemicircle(fragment)) =
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
                Classification::Decided(Start | End) => {
                    return Ok(CarrierParameterLocation::Endpoint);
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
            Classification::Decided(Start | End) => Ok(CarrierParameterLocation::Endpoint),
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
    } else if lower == Ordering::Equal || upper == Ordering::Equal {
        CarrierParameterLocation::Endpoint
    } else {
        CarrierParameterLocation::Interior
    })
}

fn parameter_range_inside_carrier(
    start: &CurveRegionParameter2,
    end: &CurveRegionParameter2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> Result<bool, CurveError> {
    let start_cmp = start.cmp_by_refinement(&carrier.start, policy)?;
    let end_cmp = end.cmp_by_refinement(&carrier.end, policy)?;
    match (start_cmp, end_cmp) {
        (Classification::Decided(start_cmp), Classification::Decided(end_cmp)) => {
            Ok(!start_cmp.is_lt() && !end_cmp.is_gt())
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Err(CurveError::Topology(format!(
                "curved-region carrier ordering remained uncertain: {reason:?}"
            )))
        }
    }
}

fn ranges_intersect(
    range: &CurveRegionParameterRange2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (start, end) = ascending_range(range, policy)?;
    Ok(!decided_parameter_cmp(end, &carrier.start, policy)?.is_lt()
        && !decided_parameter_cmp(start, &carrier.end, policy)?.is_gt())
}

fn range_inside_carrier(
    range: &CurveRegionParameterRange2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (start, end) = ascending_range(range, policy)?;
    Ok(
        !decided_parameter_cmp(start, &carrier.start, policy)?.is_lt()
            && !decided_parameter_cmp(end, &carrier.end, policy)?.is_gt(),
    )
}

fn extreme_region_parameter<const N: usize>(
    parameters: [&CurveRegionParameter2; N],
    replace_when: Ordering,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegionParameter2> {
    let mut selected = parameters[0];
    for parameter in &parameters[1..] {
        selected = match selected.cmp_by_refinement(parameter, policy) {
            Ok(Classification::Decided(order)) if order == replace_when => parameter,
            Ok(Classification::Decided(_)) => selected,
            Ok(Classification::Uncertain(reason)) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Boolean,
                    family,
                    reason,
                ));
            }
            Err(cause) => {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Boolean,
                    family,
                    cause,
                ));
            }
        };
    }
    Ok(selected.clone())
}

fn clip_corresponding_parameter_overlap(
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    correspondence: &RationalBezierOverlapParameterCorrespondence2,
    first_carrier: &RegionCarrier,
    second_carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<Option<(CurveRegionParameterRange2, CurveRegionParameterRange2)>> {
    let first_fragment = CurveRegionParameterRange2::new_validated(
        first_carrier.start.clone(),
        first_carrier.end.clone(),
    );
    let second_fragment = CurveRegionParameterRange2::new_validated(
        second_carrier.start.clone(),
        second_carrier.end.clone(),
    );
    match correspondence.clipped_curve_region_ranges(
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

fn clip_projectively_aligned_parameter_overlap(
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
    first_carrier: &RegionCarrier,
    second_carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<CarrierOverlapClip> {
    let invalid = |family, cause| ExactCurveError::invalid(CurveOperation2::Boolean, family, cause);
    let reversed = orientation == RationalBezierOverlapOrientation2::Reversed;
    let (RegionCarrierGeometry::Bezier(first_curve), RegionCarrierGeometry::Bezier(second_curve)) =
        (&first_carrier.geometry, &second_carrier.geometry)
    else {
        return clip_aligned_parameter_overlap(
            first_range,
            second_range,
            reversed,
            first_carrier,
            second_carrier,
            policy,
        );
    };
    if reversed || first_curve != second_curve {
        let first = RationalBezier2::try_from_subcurve(first_curve)
            .map_err(|cause| invalid(first_carrier.family, cause))?;
        let second = RationalBezier2::try_from_subcurve(second_curve)
            .map_err(|cause| invalid(second_carrier.family, cause))?;
        match first.same_projective_control_net_degree_aligned(&second, reversed, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(CarrierOverlapClip::Unmatched),
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Boolean,
                    first_carrier.family,
                    reason,
                ));
            }
        }
    }

    clip_aligned_parameter_overlap(
        first_range,
        second_range,
        reversed,
        first_carrier,
        second_carrier,
        policy,
    )
}

fn clip_aligned_parameter_overlap(
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    reversed: bool,
    first_carrier: &RegionCarrier,
    second_carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<CarrierOverlapClip> {
    let map_to_second = |parameter: &BezierParameter2| {
        if reversed {
            parameter.unit_complement()
        } else {
            parameter.clone()
        }
    };
    let mapped_overlap_start = map_to_second(first_range.start());
    let mapped_overlap_end = map_to_second(first_range.end());
    if decided_parameter_cmp(&mapped_overlap_start, second_range.start(), policy)?
        != Ordering::Equal
        || decided_parameter_cmp(&mapped_overlap_end, second_range.end(), policy)?
            != Ordering::Equal
    {
        return Ok(CarrierOverlapClip::Unmatched);
    }

    let (overlap_start, overlap_end) = ascending_bezier_range(first_range, policy)?;
    let overlap_start = CurveRegionParameter2::from_bezier(overlap_start.clone());
    let overlap_end = CurveRegionParameter2::from_bezier(overlap_end.clone());
    let (second_start_in_first, second_end_in_first) = if reversed {
        (
            second_carrier.end.unit_complement().ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Boolean,
                    second_carrier.family,
                    UncertaintyReason::Unsupported,
                )
            })?,
            second_carrier.start.unit_complement().ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Boolean,
                    second_carrier.family,
                    UncertaintyReason::Unsupported,
                )
            })?,
        )
    } else {
        (second_carrier.start.clone(), second_carrier.end.clone())
    };
    let start = extreme_region_parameter(
        [&overlap_start, &first_carrier.start, &second_start_in_first],
        Ordering::Less,
        first_carrier.family,
        policy,
    )?;
    let end = extreme_region_parameter(
        [&overlap_end, &first_carrier.end, &second_end_in_first],
        Ordering::Greater,
        first_carrier.family,
        policy,
    )?;
    match decided_parameter_cmp(&start, &end, policy)? {
        Ordering::Less => {}
        Ordering::Equal | Ordering::Greater => {
            return Ok(CarrierOverlapClip::Matched(None));
        }
    }
    let map_region_to_second = |parameter: &CurveRegionParameter2| {
        if reversed {
            parameter.unit_complement().ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Boolean,
                    first_carrier.family,
                    UncertaintyReason::Unsupported,
                )
            })
        } else {
            Ok(parameter.clone())
        }
    };
    let second_start = map_region_to_second(&start)?;
    let second_end = map_region_to_second(&end)?;
    Ok(CarrierOverlapClip::Matched(Some((
        CurveRegionParameterRange2::new_validated(start, end),
        CurveRegionParameterRange2::new_validated(second_start, second_end),
    ))))
}

#[cfg(test)]
pub(crate) fn clip_aligned_parameter_overlap_for_test(
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    reversed: bool,
    first_carrier_range: &CurveRegionParameterRange2,
    second_carrier_range: &CurveRegionParameterRange2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<(CurveRegionParameterRange2, CurveRegionParameterRange2)>> {
    let carrier = |operand, range: &CurveRegionParameterRange2| RegionCarrier {
        operand,
        loop_index: 0,
        fragment_index: 0,
        family: CurveFamily2::RationalBezier,
        geometry: RegionCarrierGeometry::Bezier(BezierSubcurve2::Quadratic(
            QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(
                    crate::Point2::from_values(0, 0),
                    crate::Point2::from_values(1, 0),
                )
                .expect("the test carrier line is nondegenerate"),
            ),
        )),
        start: range.start().clone(),
        end: range.end().clone(),
        reversed: false,
        filled_side_is_left: true,
        selected_fiber_endpoint_points: None,
        image_is_injective: OnceLock::new(),
        bounds: OnceLock::new(),
    };
    let first_carrier = carrier(CurveRegionBooleanOperand2::First, first_carrier_range);
    let second_carrier = carrier(CurveRegionBooleanOperand2::Second, second_carrier_range);
    match clip_aligned_parameter_overlap(
        first_range,
        second_range,
        reversed,
        &first_carrier,
        &second_carrier,
        policy,
    )? {
        CarrierOverlapClip::Matched(ranges) => Ok(ranges),
        CarrierOverlapClip::Unmatched => Err(ExactCurveError::invalid(
            CurveOperation2::Boolean,
            CurveFamily2::RationalBezier,
            CurveError::Topology("the certified test ranges were not parameter-aligned".into()),
        )),
    }
}

fn range_contains_fragment(
    range: &CurveRegionParameterRange2,
    fragment_start: &CurveRegionParameter2,
    fragment_end: &CurveRegionParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (range_start, range_end) = ascending_range(range, policy)?;
    Ok(
        !decided_parameter_cmp(fragment_start, range_start, policy)?.is_lt()
            && !decided_parameter_cmp(fragment_end, range_end, policy)?.is_gt(),
    )
}

fn ascending_range<'a>(
    range: &'a CurveRegionParameterRange2,
    policy: &CurveContext,
) -> ExactCurveResult<(&'a CurveRegionParameter2, &'a CurveRegionParameter2)> {
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

fn ascending_bezier_range<'a>(
    range: &'a BezierParameterRange2,
    policy: &CurveContext,
) -> ExactCurveResult<(&'a BezierParameter2, &'a BezierParameter2)> {
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

impl BooleanParameterOrder for CurveRegionParameter2 {
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
        | BezierSplitFragment2::AlgebraicEndpointImages { start, end, .. } => Some((start, end)),
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            Some((fragment.range().start(), fragment.range().end()))
        }
        BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
        BezierSplitFragment2::SelectedFiber(_) => None,
    }
}

const fn subcurve_family(curve: &BezierSubcurve2) -> CurveFamily2 {
    match curve {
        BezierSubcurve2::Quadratic(_) => CurveFamily2::QuadraticBezier,
        BezierSubcurve2::Cubic(_) => CurveFamily2::CubicBezier,
        BezierSubcurve2::RationalQuadratic(_) => CurveFamily2::RationalQuadraticBezier,
        BezierSubcurve2::Rational(_) => CurveFamily2::RationalBezier,
    }
}

impl RegionCarrierGeometry {
    const fn family(&self) -> CurveFamily2 {
        match self {
            Self::Bezier(curve) => subcurve_family(curve),
            Self::AlgebraicChord(_) => CurveFamily2::Line,
            Self::AnalyticParallel(_) | Self::AlgebraicCuspSemicircle(_) => {
                CurveFamily2::RationalBezier
            }
        }
    }

    fn bezier(&self) -> &BezierSubcurve2 {
        match self {
            Self::Bezier(curve) => curve,
            Self::AnalyticParallel(_) => {
                unreachable!("parallel/rational dispatch requires a Bezier carrier")
            }
            Self::AlgebraicChord(_) | Self::AlgebraicCuspSemicircle(_) => {
                unreachable!("cusp/rational dispatch requires a Bezier carrier")
            }
        }
    }

    fn parallel(&self) -> &BezierParallel2 {
        match self {
            Self::AnalyticParallel(parallel) => parallel,
            Self::Bezier(_) => {
                unreachable!("analytic pair dispatch requires a parallel carrier")
            }
            Self::AlgebraicChord(_) | Self::AlgebraicCuspSemicircle(_) => {
                unreachable!("cusp/parallel dispatch requires a parallel carrier")
            }
        }
    }

    fn algebraic_cusp(&self) -> &crate::BezierAlgebraicCuspSemicircleFragment2 {
        match self {
            Self::AlgebraicCuspSemicircle(fragment) => fragment,
            Self::Bezier(_) | Self::AnalyticParallel(_) | Self::AlgebraicChord(_) => {
                unreachable!("algebraic-cusp dispatch requires a cusp carrier")
            }
        }
    }

    fn point_at(
        &self,
        parameter: &crate::Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::Point2>> {
        match self {
            Self::Bezier(curve) => Ok(curve.point_at(parameter, policy)),
            Self::AnalyticParallel(parallel) => parallel.point_at(parameter, policy),
            Self::AlgebraicChord(chord) => match chord.exact_line() {
                Some(line) => Ok(Classification::Decided(line.point_at(parameter.clone()))),
                None => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            },
            Self::AlgebraicCuspSemicircle(fragment) => {
                Ok(match fragment.semicircle().point_at(parameter, policy)? {
                    Classification::Decided(point) => point.exact_rational_point(policy).map_or(
                        Classification::Uncertain(UncertaintyReason::Unsupported),
                        Classification::Decided,
                    ),
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                })
            }
        }
    }

    fn derivative_at(
        &self,
        parameter: &crate::Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveDerivative2>> {
        match self {
            Self::Bezier(curve) => RationalBezier2::try_from_subcurve(curve)
                .map(|curve| curve.derivative_at_classified(parameter, policy)),
            Self::AnalyticParallel(parallel) => parallel.derivative_at(parameter, policy),
            Self::AlgebraicChord(chord) => match chord.exact_line() {
                Some(line) => Ok(Classification::Decided(CurveDerivative2::new(
                    line.end().x() - line.start().x(),
                    line.end().y() - line.start().y(),
                ))),
                None => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            },
            Self::AlgebraicCuspSemicircle(_) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        }
    }

    fn certified_outer_bounds(&self, policy: &CurveContext) -> Classification<Aabb2> {
        match self {
            Self::Bezier(curve) => subcurve_certified_outer_bounds(curve, policy),
            Self::AnalyticParallel(parallel) => match parallel.conservative_bounds(policy) {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
            Self::AlgebraicChord(chord) => match chord.conservative_bounds(policy) {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
            Self::AlgebraicCuspSemicircle(fragment) => match fragment.conservative_bounds() {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
        }
    }

    fn certified_outer_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        match self {
            Self::Bezier(curve) => subcurve_certified_outer_bounds(curve, policy),
            Self::AnalyticParallel(parallel) => match parallel.conservative_bounds(policy) {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
            Self::AlgebraicChord(chord) => {
                match chord.conservative_bounds_refined(refinement_steps, policy) {
                    Ok(bounds) => bounds,
                    Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
                }
            }
            Self::AlgebraicCuspSemicircle(fragment) => match fragment
                .semicircle()
                .conservative_bounds_refined(refinement_steps, policy)
            {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
        }
    }

    fn has_certified_injective_axis(&self, policy: &CurveContext) -> bool {
        match self {
            Self::Bezier(curve) => curve.has_certified_injective_axis(policy),
            Self::AnalyticParallel(parallel) => {
                parallel.regular_fragment_has_certified_injective_axis(policy)
                    || matches!(
                        parallel.exact_rational_parallel_component(policy),
                        Ok(Classification::Decided(Some(curve)))
                            if curve.has_certified_injective_axis(policy)
                    )
            }
            Self::AlgebraicChord(_) => false,
            Self::AlgebraicCuspSemicircle(_) => false,
        }
    }

    fn has_certified_injective_image(&self, policy: &CurveContext) -> bool {
        match self {
            Self::Bezier(curve) => curve.has_certified_injective_image(policy),
            Self::AnalyticParallel(_) => self.has_certified_injective_axis(policy),
            Self::AlgebraicChord(_) => true,
            Self::AlgebraicCuspSemicircle(_) => true,
        }
    }

    fn exact_rational_component(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalBezier2>>> {
        match self {
            Self::Bezier(curve) => RationalBezier2::try_from_subcurve(curve)
                .map(Some)
                .map(Classification::Decided),
            Self::AnalyticParallel(parallel) => parallel.exact_rational_parallel_component(policy),
            Self::AlgebraicChord(_) => Ok(Classification::Decided(None)),
            Self::AlgebraicCuspSemicircle(_) => Ok(Classification::Decided(None)),
        }
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
mod certified_successor_tests {
    use super::*;
    use crate::bezier_offset::{
        BezierAlgebraicChordAxisDirection2, BezierAlgebraicCuspSemicircle2,
        BezierAlgebraicCuspSemicirclePairIntersections2, BezierAlgebraicCuspSemicircleParameter2,
        exact_selected_fiber_parameter_for_test, nonlinear_parameter_component_overlap_for_test,
    };
    use crate::{BezierAlgebraicCuspSemicircleFragment2, CubicBezier2};
    use crate::{
        BezierAlgebraicParameter2, BezierParallelFragment2, CurvePath2, CurveRegionBoundaryLoop2,
        LineSeg2, Point2, QuadraticBezier2, RationalBezier2, RationalBezierAlgebraicPointImage2,
        Real,
    };
    use num::bigint::{BigInt, BigUint};

    fn decided<T>(classification: Classification<T>) -> T {
        match classification {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                panic!("classification unexpectedly uncertain: {reason:?}")
            }
        }
    }

    fn carrier_parameter(parameter: BezierParameter2) -> CurveRegionParameter2 {
        CurveRegionParameter2::from_bezier(parameter)
    }

    fn carrier_range(range: &BezierParameterRange2) -> CurveRegionParameterRange2 {
        CurveRegionParameterRange2::from_bezier_range(range.clone())
    }

    fn sqrt_half_parameter(policy: &CurveContext) -> BezierAlgebraicParameter2 {
        let polynomial = decided(
            crate::BezierParameterPolynomial::try_new_power_basis(
                vec![(-1).into(), 0.into(), 2.into()],
                policy,
            )
            .expect("valid parameter polynomial"),
        );
        let interval = decided(
            crate::BezierParameterInterval::try_new(
                (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
                Real::one(),
                policy,
            )
            .expect("valid parameter interval"),
        );
        decided(
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)
                .expect("isolated parameter"),
        )
    }

    #[test]
    fn selected_parameter_component_overlap_clips_in_both_boolean_orders() {
        let fraction = |numerator: i8, denominator: i8| {
            (Real::from(numerator) / Real::from(denominator)).unwrap()
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let retained = sqrt_half_parameter(&policy);
            let selected =
                |value: Real| {
                    CurveRegionParameter2::from_selected_fiber(
                        exact_selected_fiber_parameter_for_test(retained.clone(), value, &policy),
                    )
                };
            let selected_range = |start: Real, end: Real| {
                CurveRegionParameterRange2::new_validated(selected(start), selected(end))
            };
            let ordinary_range = |start: Real, end: Real| {
                CurveRegionParameterRange2::from_bezier_range(BezierParameterRange2::from_exact(
                    start, end,
                ))
            };
            let source = nonlinear_parameter_component_overlap_for_test(&policy);
            let first_overlap = CurveRegionParameterRange2::from_bezier_range(
                source.overlap().first_range().clone(),
            );
            let second_overlap = CurveRegionParameterRange2::from_bezier_range(
                source.overlap().second_range().clone(),
            );
            let line = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                    .expect("the test carrier is nondegenerate"),
            ));
            let clip = |first_range: CurveRegionParameterRange2,
                        second_range: CurveRegionParameterRange2,
                        swapped: bool| {
                let carrier = |operand, range: &CurveRegionParameterRange2| {
                    let geometry = RegionCarrierGeometry::Bezier(line.clone());
                    RegionCarrier {
                        operand,
                        loop_index: 0,
                        fragment_index: 0,
                        family: geometry.family(),
                        geometry,
                        start: range.start().clone(),
                        end: range.end().clone(),
                        reversed: false,
                        filled_side_is_left: true,
                        selected_fiber_endpoint_points: None,
                        image_is_injective: OnceLock::new(),
                        bounds: OnceLock::new(),
                    }
                };
                let empty_first = CurveRegion2::empty();
                let empty_second = CurveRegion2::empty();
                let context = CurveRegionBooleanContext {
                    data: CurveRegionBooleanContextData {
                        first: &empty_first,
                        second: &empty_second,
                        policy,
                        carriers: vec![
                            carrier(CurveRegionBooleanOperand2::First, &first_range),
                            carrier(CurveRegionBooleanOperand2::Second, &second_range),
                        ],
                        first_carrier_count: 1,
                        authored_carrier_pair_count: 1,
                        pairs: Vec::new(),
                        bezier_self_intersections: Vec::new(),
                        parallel_self_intersections: Vec::new(),
                        strict_line_image_only: OnceLock::new(),
                    },
                };
                let pair = RegionCarrierPair {
                    first_carrier_index: 0,
                    second_carrier_index: 1,
                    context: RegionCarrierPairContext::ParallelPair,
                };
                let (first_range, second_range) = if swapped {
                    (second_overlap.clone(), first_overlap.clone())
                } else {
                    (first_overlap.clone(), second_overlap.clone())
                };
                let overlap = RegionPairOverlap {
                    source: Some(RegionPairOverlapSource::ParameterComponent {
                        source: source.clone(),
                        swapped,
                    }),
                    first_range,
                    second_range,
                    orientation: source.overlap().orientation(),
                };
                context
                    .clipped_overlap_ranges(&pair, &overlap)
                    .expect("selected component clipping must decide")
                    .expect("the selected carrier ranges overlap")
            };
            let assert_selected = |parameter: &CurveRegionParameter2, expected: Real| {
                assert_eq!(
                    parameter
                        .as_selected_fiber()
                        .expect("an unchanged component bound must remain selected")
                        .order_to_real(&expected, &policy)
                        .unwrap(),
                    Classification::Decided(Ordering::Equal),
                );
            };

            let (first, second) = clip(
                selected_range(fraction(1, 4), fraction(3, 4)),
                ordinary_range(fraction(1, 16), fraction(1, 4)),
                false,
            );
            assert_selected(first.start(), fraction(1, 4));
            assert_eq!(first.end().as_exact(), Some(&fraction(1, 2)));
            assert_eq!(
                second.exact_endpoints(),
                Some((&fraction(1, 16), &fraction(1, 4))),
            );

            let (first, second) = clip(
                selected_range(fraction(1, 16), fraction(1, 4)),
                ordinary_range(fraction(1, 4), fraction(3, 4)),
                true,
            );
            assert_selected(first.start(), fraction(1, 16));
            assert_selected(first.end(), fraction(1, 4));
            assert_eq!(second.start().as_exact(), Some(&fraction(1, 4)));
            assert_eq!(second.end().as_exact(), Some(&fraction(1, 2)));
        }
    }

    #[test]
    fn selected_projective_overlap_clips_without_global_parameter_projection() {
        let fraction = |numerator: i8, denominator: i8| {
            (Real::from(numerator) / Real::from(denominator)).unwrap()
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let retained = sqrt_half_parameter(&policy);
            let selected =
                |value: Real| {
                    CurveRegionParameter2::from_selected_fiber(
                        exact_selected_fiber_parameter_for_test(retained.clone(), value, &policy),
                    )
                };
            let first_carrier_range = CurveRegionParameterRange2::new_validated(
                selected(fraction(1, 2)),
                selected(fraction(3, 4)),
            );
            let second_carrier_range = CurveRegionParameterRange2::new_validated(
                selected(fraction(1, 3)),
                selected(fraction(1, 2)),
            );
            let controls = vec![
                Point2::from_values(0, 0),
                Point2::from_values(1, 1),
                Point2::from_values(2, 0),
            ];
            let first = RationalBezier2::try_new(controls.clone(), vec![Real::one(); 3])
                .expect("valid polynomial quadratic");
            // Scaling homogeneous Bernstein control i by 2^i composes the
            // first parameter with t=2s/(1+s).
            let second = RationalBezier2::try_new(
                controls,
                vec![Real::one(), Real::from(2_i8), Real::from(4_i8)],
            )
            .expect("valid projectively reparameterized quadratic");
            let first_range = BezierParameterRange2::from_exact(Real::zero(), Real::one());
            let second_range = BezierParameterRange2::from_exact(Real::zero(), Real::one());
            let overlap = RationalBezierIntersectionOverlap2::from_certified_parameters(
                first_range.start().clone(),
                first_range.end().clone(),
                second_range.start().clone(),
                second_range.end().clone(),
                RationalBezierOverlapOrientation2::Same,
                [true, true],
            );
            let endpoint_correspondence =
                RationalBezierOverlapParameterCorrespondence2::for_overlap(
                    &first, &second, &overlap, &policy,
                );
            let carrier = |operand, curve: RationalBezier2, range: &CurveRegionParameterRange2| {
                let geometry = RegionCarrierGeometry::Bezier(BezierSubcurve2::Rational(curve));
                RegionCarrier {
                    operand,
                    loop_index: 0,
                    fragment_index: 0,
                    family: geometry.family(),
                    geometry,
                    start: range.start().clone(),
                    end: range.end().clone(),
                    reversed: false,
                    filled_side_is_left: true,
                    selected_fiber_endpoint_points: None,
                    image_is_injective: OnceLock::new(),
                    bounds: OnceLock::new(),
                }
            };
            for correspondence in [
                endpoint_correspondence,
                RationalBezierOverlapParameterCorrespondence2::RangeProjective {
                    second_to_first_scale: Real::from(2_i8),
                    reversed: false,
                },
            ] {
                assert!(correspondence.projective_reversal().is_none());
                let first_carrier = carrier(
                    CurveRegionBooleanOperand2::First,
                    first.clone(),
                    &first_carrier_range,
                );
                let second_carrier = carrier(
                    CurveRegionBooleanOperand2::Second,
                    second.clone(),
                    &second_carrier_range,
                );
                let (first, second) = clip_corresponding_parameter_overlap(
                    &first_range,
                    &second_range,
                    &correspondence,
                    &first_carrier,
                    &second_carrier,
                    &policy,
                )
                .expect("selected projective clipping must decide")
                .expect("the selected projective subranges overlap");
                for (parameter, expected) in [
                    (first.start(), fraction(1, 2)),
                    (first.end(), fraction(2, 3)),
                    (second.start(), fraction(1, 3)),
                    (second.end(), fraction(1, 2)),
                ] {
                    assert_eq!(
                        parameter
                            .as_selected_fiber()
                            .expect("projective clipping must retain a local selected scalar")
                            .order_to_real(&expected, &policy)
                            .unwrap(),
                        Classification::Decided(Ordering::Equal),
                    );
                }
            }
        }
    }

    #[test]
    fn selected_general_overlap_clips_through_exact_projection() {
        let fraction = |numerator: i8, denominator: i8| {
            (Real::from(numerator) / Real::from(denominator)).unwrap()
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let retained = sqrt_half_parameter(&policy);
            let selected =
                |value: Real| {
                    CurveRegionParameter2::from_selected_fiber(
                        exact_selected_fiber_parameter_for_test(retained.clone(), value, &policy),
                    )
                };
            let first_carrier_range = CurveRegionParameterRange2::new_validated(
                selected(fraction(1, 4)),
                selected(fraction(3, 4)),
            );
            let second_carrier_range = CurveRegionParameterRange2::new_validated(
                selected(fraction(1, 16)),
                selected(fraction(1, 4)),
            );
            // The first line uses x=t^2 while the second uses x=u. Their
            // positive-dimensional image correspondence u=t^2 is neither
            // affine nor projective.
            let first = RationalBezier2::try_new(
                vec![
                    Point2::from_values(0, 0),
                    Point2::from_values(0, 0),
                    Point2::from_values(1, 0),
                ],
                vec![Real::one(); 3],
            )
            .expect("valid quadratically parameterized line");
            let second = RationalBezier2::try_new(
                vec![Point2::from_values(0, 0), Point2::from_values(1, 0)],
                vec![Real::one(); 2],
            )
            .expect("valid affine line");
            let correspondence = RationalBezierOverlapParameterCorrespondence2::General {
                first: first.clone(),
                second: second.clone(),
                unresolved: None,
            };
            let carrier = |operand, curve: RationalBezier2, range: &CurveRegionParameterRange2| {
                let geometry = RegionCarrierGeometry::Bezier(BezierSubcurve2::Rational(curve));
                RegionCarrier {
                    operand,
                    loop_index: 0,
                    fragment_index: 0,
                    family: geometry.family(),
                    geometry,
                    start: range.start().clone(),
                    end: range.end().clone(),
                    reversed: false,
                    filled_side_is_left: true,
                    selected_fiber_endpoint_points: None,
                    image_is_injective: OnceLock::new(),
                    bounds: OnceLock::new(),
                }
            };
            let first_carrier = carrier(
                CurveRegionBooleanOperand2::First,
                first,
                &first_carrier_range,
            );
            let second_carrier = carrier(
                CurveRegionBooleanOperand2::Second,
                second,
                &second_carrier_range,
            );
            let first_range = BezierParameterRange2::from_exact(Real::zero(), Real::one());
            let second_range = BezierParameterRange2::from_exact(Real::zero(), Real::one());
            let (first, second) = clip_corresponding_parameter_overlap(
                &first_range,
                &second_range,
                &correspondence,
                &first_carrier,
                &second_carrier,
                &policy,
            )
            .expect("selected general clipping must decide")
            .expect("the selected nonlinear subranges overlap");
            assert_eq!(
                first
                    .start()
                    .as_selected_fiber()
                    .expect("an unchanged first bound must remain selected")
                    .order_to_real(&fraction(1, 4), &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Equal),
            );
            assert_eq!(first.end().as_exact(), Some(&fraction(1, 2)));
            for (parameter, expected) in [
                (second.start(), fraction(1, 16)),
                (second.end(), fraction(1, 4)),
            ] {
                assert_eq!(
                    parameter
                        .as_selected_fiber()
                        .expect("an unchanged second bound must remain selected")
                        .order_to_real(&expected, &policy)
                        .unwrap(),
                    Classification::Decided(Ordering::Equal),
                );
            }
        }
    }

    fn sqrt_third_parameter(policy: &CurveContext) -> BezierAlgebraicParameter2 {
        let polynomial = decided(
            crate::BezierParameterPolynomial::try_new_power_basis(
                vec![(-1).into(), 0.into(), 3.into()],
                policy,
            )
            .expect("valid parameter polynomial"),
        );
        let interval = decided(
            crate::BezierParameterInterval::try_new(
                (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
                Real::one(),
                policy,
            )
            .expect("valid parameter interval"),
        );
        decided(
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)
                .expect("isolated parameter"),
        )
    }

    fn sqrt_reciprocal_parameter(
        denominator: i8,
        policy: &CurveContext,
    ) -> BezierAlgebraicParameter2 {
        let polynomial = decided(
            crate::BezierParameterPolynomial::try_new_power_basis(
                vec![(-1).into(), 0.into(), denominator.into()],
                policy,
            )
            .expect("valid reciprocal-square-root parameter polynomial"),
        );
        let interval = decided(
            crate::BezierParameterInterval::try_new(
                (Real::one() / Real::from(4_i8)).expect("nonzero denominator"),
                (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
                policy,
            )
            .expect("valid reciprocal-square-root parameter interval"),
        );
        decided(
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)
                .expect("isolated reciprocal-square-root parameter"),
        )
    }

    fn rational_line(start_x: i32, end_x: i32) -> RationalBezier2 {
        RationalBezier2::try_new(
            vec![
                Point2::from_values(start_x, 0),
                Point2::from_values(end_x, 0),
            ],
            vec![Real::one(); 2],
        )
        .expect("valid rational line")
    }

    fn algebraic_chord_carrier(
        operand: CurveRegionBooleanOperand2,
        chord: crate::BezierAlgebraicChord2,
    ) -> RegionCarrier {
        RegionCarrier {
            operand,
            loop_index: 0,
            fragment_index: 0,
            family: CurveFamily2::Line,
            start: CurveRegionParameter2::from_algebraic_chord(chord.start_parameter()),
            end: CurveRegionParameter2::from_algebraic_chord(chord.end_parameter()),
            geometry: RegionCarrierGeometry::AlgebraicChord(chord),
            reversed: false,
            filled_side_is_left: true,
            selected_fiber_endpoint_points: None,
            image_is_injective: OnceLock::new(),
            bounds: OnceLock::new(),
        }
    }

    fn selected_field_algebraic_chord_rectangle(policy: &CurveContext) -> CurveRegion2 {
        let parameter = sqrt_half_parameter(policy);
        let point = |positive: bool, height: i32| {
            let endpoint_x = if positive { 1 } else { -1 };
            RationalBezierIntersectionPointEvidence2::Algebraic(
                RationalBezier2::try_new(
                    vec![
                        Point2::from_values(0, height),
                        Point2::from_values(endpoint_x, height),
                    ],
                    vec![Real::one(); 2],
                )
                .expect("valid selected-field line")
                .point_at_algebraic_parameter(&parameter, policy)
                .expect("selected-field endpoint"),
            )
        };
        let bottom_left = point(false, 0);
        let bottom_right = point(true, 0);
        let top_right = point(true, 1);
        let top_left = point(false, 1);
        let chord = |start, end| {
            BezierSplitFragment2::AlgebraicChord(decided(
                crate::BezierAlgebraicChord2::try_new(start, end, policy)
                    .expect("valid retained chord"),
            ))
        };
        let boundary = CurveRegionBoundaryLoop2::new(
            vec![
                chord(bottom_left.clone(), bottom_right.clone()),
                chord(bottom_right, top_right.clone()),
                chord(top_right, top_left.clone()),
                chord(top_left, bottom_left),
            ],
            policy,
        )
        .expect("valid selected-field rectangle");
        CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::CurveBoundaryInteriorSide2::Left],
        )
        .expect("valid selected-field region")
    }

    #[test]
    fn cusp_chord_pair_retains_an_interior_axis_contact() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parameter = sqrt_half_parameter(&policy);
            let point = |x: Vec<Real>, y: Vec<Real>| {
                RationalBezierIntersectionPointEvidence2::Algebraic(
                    RationalBezierAlgebraicPointImage2::from_retained_expression(
                        parameter.clone(),
                        crate::bezier_algebraic_image::parameter_representation(
                            &parameter, &policy,
                        ),
                        x,
                        y,
                        vec![Real::one()],
                        "test region cusp/chord point",
                    ),
                )
            };
            let center = point(vec![Real::zero(), Real::one()], vec![Real::zero()]);
            let semicircle = decided(
                BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                    &center,
                    (1, 0),
                    Real::from(2_i8),
                    false,
                    &policy,
                )
                .expect("valid selected circle"),
            )
            .expect("nonzero circle radius");
            let chord = crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
                point(vec![Real::zero(), Real::one()], vec![Real::from(-3_i8)]),
                point(vec![Real::zero(), Real::one()], vec![Real::from(3_i8)]),
                BezierAlgebraicChordAxisDirection2::PositiveY,
                &policy,
            );
            let cusp = cusp_test_carrier(
                semicircle,
                Real::zero(),
                Real::one(),
                CurveRegionBooleanOperand2::First,
                &policy,
            );
            let empty_first = CurveRegion2::empty();
            let empty_second = CurveRegion2::empty();
            let pair = RegionCarrierPair {
                first_carrier_index: 0,
                second_carrier_index: 1,
                context: RegionCarrierPairContext::CuspChord {
                    cusp_is_first: true,
                },
            };
            let context = CurveRegionBooleanContext {
                data: CurveRegionBooleanContextData {
                    first: &empty_first,
                    second: &empty_second,
                    policy,
                    carriers: vec![
                        cusp,
                        algebraic_chord_carrier(CurveRegionBooleanOperand2::Second, chord.clone()),
                    ],
                    first_carrier_count: 1,
                    authored_carrier_pair_count: 1,
                    pairs: vec![pair],
                    bezier_self_intersections: Vec::new(),
                    parallel_self_intersections: Vec::new(),
                    strict_line_image_only: OnceLock::new(),
                },
            };
            let result = context
                .pair_result(&context.data.pairs[0])
                .expect("axis cusp/chord pair must complete");
            assert!(result.blockers.is_empty(), "{result:?}");
            let [contact] = result.contacts.as_slice() else {
                panic!("expected one retained cusp/chord contact: {result:?}");
            };
            assert!(contact.is_certified_transverse());
            assert_eq!(contact.tangent_cross_sign, Some(RealSign::Negative));
            assert!(matches!(
                contact.point(),
                Some(RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_))
            ));
            let cusp_parameter = contact
                .first_parameter()
                .as_algebraic_cusp()
                .expect("first carrier must retain the cusp parameter");
            let half = (Real::one() / Real::from(2_i8)).unwrap();
            assert_eq!(
                cusp_parameter.order_to_real(&half, &policy).unwrap(),
                Classification::Decided(Ordering::Equal),
            );
            let chord_parameter = contact
                .second_parameter()
                .as_algebraic_chord()
                .expect("second carrier must retain the chord parameter");
            assert_eq!(
                chord_parameter
                    .cmp_by_refinement(&chord.start_parameter(), &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Greater),
            );
            assert_eq!(
                chord_parameter
                    .cmp_by_refinement(&chord.end_parameter(), &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Less),
            );
            let evidence = context
                .build_intersection_evidence()
                .expect("axis cusp/chord contact must enter region evidence");
            assert!(evidence.is_complete(), "{evidence:?}");
            assert_eq!(evidence.contacts().len(), 1, "{evidence:?}");
        }
    }

    #[test]
    fn cusp_chord_pair_retains_an_interior_exact_oblique_contact() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parameter = sqrt_half_parameter(&policy);
            let center = RationalBezierIntersectionPointEvidence2::Algebraic(
                RationalBezierAlgebraicPointImage2::from_retained_expression(
                    parameter.clone(),
                    crate::bezier_algebraic_image::parameter_representation(&parameter, &policy),
                    vec![Real::zero(), Real::one()],
                    vec![Real::zero()],
                    vec![Real::one()],
                    "test region oblique cusp/chord center",
                ),
            );
            let semicircle = decided(
                BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                    &center,
                    (1, 0),
                    Real::from(2_i8),
                    false,
                    &policy,
                )
                .expect("valid selected circle"),
            )
            .expect("nonzero circle radius");
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(-3, -3)),
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(3, 3)),
                    &policy,
                )
                .expect("valid exact oblique chord"),
            );
            let cusp = cusp_test_carrier(
                semicircle,
                Real::zero(),
                Real::one(),
                CurveRegionBooleanOperand2::First,
                &policy,
            );
            let empty_first = CurveRegion2::empty();
            let empty_second = CurveRegion2::empty();
            let pair = RegionCarrierPair {
                first_carrier_index: 0,
                second_carrier_index: 1,
                context: RegionCarrierPairContext::CuspChord {
                    cusp_is_first: true,
                },
            };
            let context = CurveRegionBooleanContext {
                data: CurveRegionBooleanContextData {
                    first: &empty_first,
                    second: &empty_second,
                    policy,
                    carriers: vec![
                        cusp,
                        algebraic_chord_carrier(CurveRegionBooleanOperand2::Second, chord.clone()),
                    ],
                    first_carrier_count: 1,
                    authored_carrier_pair_count: 1,
                    pairs: vec![pair],
                    bezier_self_intersections: Vec::new(),
                    parallel_self_intersections: Vec::new(),
                    strict_line_image_only: OnceLock::new(),
                },
            };
            let result = context
                .pair_result(&context.data.pairs[0])
                .expect("exact oblique cusp/chord pair must complete");
            assert!(result.blockers.is_empty(), "{result:?}");
            let [contact] = result.contacts.as_slice() else {
                panic!("expected one retained oblique cusp/chord contact: {result:?}");
            };
            assert!(contact.is_certified_transverse());
            assert_eq!(contact.tangent_cross_sign, Some(RealSign::Negative));
            let point = contact
                .point()
                .expect("the exact oblique contact must retain point evidence");
            assert!(matches!(
                point,
                RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_)
            ));
            assert_eq!(
                chord.contains_point_evidence(point, &policy).unwrap(),
                Classification::Decided(true),
            );
            assert_eq!(
                context.data.carriers[0]
                    .geometry
                    .algebraic_cusp()
                    .contains_point_evidence(point, &policy)
                    .unwrap(),
                Classification::Decided(true),
            );
            let chord_parameter = contact
                .second_parameter()
                .as_algebraic_chord()
                .expect("the second carrier must retain an oblique chord parameter");
            assert_eq!(
                chord_parameter
                    .cmp_by_refinement(&chord.start_parameter(), &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Greater),
            );
            assert_eq!(
                chord_parameter
                    .cmp_by_refinement(&chord.end_parameter(), &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Less),
            );
            let evidence = context
                .build_intersection_evidence()
                .expect("the exact oblique contact must enter region evidence");
            assert!(evidence.is_complete(), "{evidence:?}");
            assert_eq!(evidence.contacts().len(), 1, "{evidence:?}");
        }
    }

    #[test]
    fn cusp_chord_pair_retains_an_independent_field_oblique_contact() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let center_parameter = sqrt_half_parameter(&policy);
            let center = RationalBezierIntersectionPointEvidence2::Algebraic(
                RationalBezierAlgebraicPointImage2::from_retained_expression(
                    center_parameter.clone(),
                    crate::bezier_algebraic_image::parameter_representation(
                        &center_parameter,
                        &policy,
                    ),
                    vec![Real::zero(), Real::one()],
                    vec![Real::zero()],
                    vec![Real::one()],
                    "test region independent-field oblique circle center",
                ),
            );
            let semicircle = decided(
                BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                    &center,
                    (1, 0),
                    Real::from(2_i8),
                    false,
                    &policy,
                )
                .expect("valid selected circle"),
            )
            .expect("nonzero circle radius");
            let endpoint = |parameter: &BezierAlgebraicParameter2, start: Point2, end: Point2| {
                let curve =
                    RationalBezier2::try_new(vec![start, end], vec![Real::one(), Real::one()])
                        .expect("valid endpoint carrier");
                RationalBezierIntersectionPointEvidence2::Algebraic(
                    curve
                        .point_at_algebraic_parameter(parameter, &policy)
                        .expect("valid endpoint image"),
                )
            };
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    endpoint(
                        &sqrt_reciprocal_parameter(5, &policy),
                        Point2::from_values(-3, -3),
                        Point2::from_values(-2, -3),
                    ),
                    endpoint(
                        &sqrt_reciprocal_parameter(7, &policy),
                        Point2::from_values(3, 3),
                        Point2::from_values(3, 4),
                    ),
                    &policy,
                )
                .expect("valid independent-field oblique chord"),
            );
            assert!(chord.exact_line().is_none());
            let cusp = cusp_test_carrier(
                semicircle,
                Real::zero(),
                Real::one(),
                CurveRegionBooleanOperand2::First,
                &policy,
            );
            let empty_first = CurveRegion2::empty();
            let empty_second = CurveRegion2::empty();
            let pair = RegionCarrierPair {
                first_carrier_index: 0,
                second_carrier_index: 1,
                context: RegionCarrierPairContext::CuspChord {
                    cusp_is_first: true,
                },
            };
            let context = CurveRegionBooleanContext {
                data: CurveRegionBooleanContextData {
                    first: &empty_first,
                    second: &empty_second,
                    policy,
                    carriers: vec![
                        cusp,
                        algebraic_chord_carrier(CurveRegionBooleanOperand2::Second, chord.clone()),
                    ],
                    first_carrier_count: 1,
                    authored_carrier_pair_count: 1,
                    pairs: vec![pair],
                    bezier_self_intersections: Vec::new(),
                    parallel_self_intersections: Vec::new(),
                    strict_line_image_only: OnceLock::new(),
                },
            };
            let result = context
                .pair_result(&context.data.pairs[0])
                .expect("independent-field oblique cusp/chord pair must complete");
            assert!(result.blockers.is_empty(), "{result:?}");
            let [contact] = result.contacts.as_slice() else {
                panic!("expected one independent-field oblique contact: {result:?}");
            };
            assert!(contact.is_certified_transverse());
            assert_eq!(contact.tangent_cross_sign, Some(RealSign::Negative));
            assert!(matches!(
                contact.point(),
                Some(RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_))
            ));
            let chord_parameter = contact
                .second_parameter()
                .as_algebraic_chord()
                .expect("the second carrier must retain an oblique chord parameter");
            assert_eq!(
                chord_parameter
                    .cmp_by_refinement(&chord.start_parameter(), &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Greater),
            );
            assert_eq!(
                chord_parameter
                    .cmp_by_refinement(&chord.end_parameter(), &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Less),
            );
            let evidence = context
                .build_intersection_evidence()
                .expect("the independent-field oblique contact must enter region evidence");
            assert!(evidence.is_complete(), "{evidence:?}");
            assert_eq!(evidence.contacts().len(), 1, "{evidence:?}");
        }
    }

    #[test]
    fn shared_chord_splits_contacts_from_independent_selected_circles_in_order() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first_parameter = sqrt_half_parameter(&policy);
            let second_parameter = sqrt_third_parameter(&policy);
            let center = |parameter: &BezierAlgebraicParameter2, label| {
                RationalBezierIntersectionPointEvidence2::Algebraic(
                    RationalBezierAlgebraicPointImage2::from_retained_expression(
                        parameter.clone(),
                        crate::bezier_algebraic_image::parameter_representation(parameter, &policy),
                        vec![Real::zero(), Real::one()],
                        vec![Real::zero()],
                        vec![Real::one()],
                        label,
                    ),
                )
            };
            let circle = |parameter: &BezierAlgebraicParameter2, label| {
                decided(
                    BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                        &center(parameter, label),
                        (1, 0),
                        Real::from(2_i8),
                        false,
                        &policy,
                    )
                    .expect("valid independent selected circle"),
                )
                .expect("nonzero independent selected circle")
            };
            let chord = crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
                RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(-3, 1)),
                RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(3, 1)),
                BezierAlgebraicChordAxisDirection2::PositiveX,
                &policy,
            );
            let contacts = |circle: &BezierAlgebraicCuspSemicircle2| -> [CurveRegionParameter2; 2] {
                let Classification::Decided(
                    BezierAlgebraicCuspSemicircleRetainedChordIntersections2::Contacts(contacts),
                ) = circle.chord_intersections(&chord, &policy).unwrap()
                else {
                    panic!("both independent circle contacts must be retained");
                };
                assert_eq!(contacts.len(), 2);
                std::array::from_fn(|index| {
                    CurveRegionParameter2::from_algebraic_chord(
                        contacts[index].chord_parameter.clone(),
                    )
                })
            };
            let first = contacts(&circle(
                &first_parameter,
                "first shared-chord selected circle center",
            ));
            let second = contacts(&circle(
                &second_parameter,
                "second shared-chord selected circle center",
            ));
            let carrier =
                algebraic_chord_carrier(CurveRegionBooleanOperand2::Second, chord.clone());
            let event = |parameter, topology_vertex| CarrierEvent {
                parameter,
                topology_vertex: Some(topology_vertex),
            };
            let events = [
                event(carrier.end.clone(), 6),
                event(first[1].clone(), 5),
                event(second[0].clone(), 1),
                event(carrier.start.clone(), 0),
                event(second[1].clone(), 4),
                event(first[0].clone(), 2),
            ];
            let fragments = split_algebraic_chord_carrier(&carrier, &chord, &events, &policy)
                .expect("independent selected-circle contacts must split the shared chord");
            assert_eq!(fragments.len(), 5);
            assert_eq!(
                fragments
                    .iter()
                    .map(|fragment| (fragment.start_topology_vertex, fragment.end_topology_vertex,))
                    .collect::<Vec<_>>(),
                vec![
                    (Some(0), Some(1)),
                    (Some(1), Some(2)),
                    (Some(2), Some(4)),
                    (Some(4), Some(5)),
                    (Some(5), Some(6)),
                ],
            );
        }
    }

    #[test]
    fn algebraic_chord_carrier_retains_ordered_interior_splits() {
        let policy = CurveContext::STRICT;
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(0, 0)),
                RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(4, 0)),
                &policy,
            )
            .expect("valid retained chord"),
        );
        let geometry = RegionCarrierGeometry::AlgebraicChord(chord.clone());
        let carrier = RegionCarrier {
            operand: CurveRegionBooleanOperand2::First,
            loop_index: 0,
            fragment_index: 0,
            family: geometry.family(),
            geometry,
            start: CurveRegionParameter2::from_algebraic_chord(chord.start_parameter()),
            end: CurveRegionParameter2::from_algebraic_chord(chord.end_parameter()),
            reversed: false,
            filled_side_is_left: true,
            selected_fiber_endpoint_points: None,
            image_is_injective: OnceLock::new(),
            bounds: OnceLock::new(),
        };
        let cut = |x, vertex| CarrierEvent {
            parameter: CurveRegionParameter2::from_algebraic_chord(
                decided(
                    chord
                        .parameter_at_certified_point(
                            RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(
                                x, 0,
                            )),
                            &policy,
                        )
                        .expect("certified chord point"),
                )
                .expect("the cut lies on the chord"),
            ),
            topology_vertex: Some(vertex),
        };
        let events = vec![cut(4, 4), cut(1, 1), cut(0, 0), cut(3, 3)];
        let splits = split_algebraic_chord_carrier(&carrier, &chord, &events, &policy)
            .expect("interior chord cuts must remain exact");
        assert_eq!(splits.len(), 3);
        assert_eq!(
            splits
                .iter()
                .map(|split| (split.start_topology_vertex, split.end_topology_vertex))
                .collect::<Vec<_>>(),
            vec![(Some(0), Some(1)), (Some(1), Some(3)), (Some(3), Some(4))]
        );
        for split in splits {
            assert!(matches!(
                split.fragment,
                BezierSplitFragment2::AlgebraicChord(_)
            ));
        }
    }

    #[test]
    fn exact_retained_chord_uses_the_canonical_boolean_carrier() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(0, 0)),
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(2, 0)),
                    &policy,
                )
                .expect("valid exact retained chord"),
            );
            let curved = CubicBezier2::new(
                Point2::from_values(2, 0),
                Point2::from_values(2, 2),
                Point2::from_values(0, 2),
                Point2::from_values(0, 0),
            );
            let boundary = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::AlgebraicChord(chord),
                    BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: BezierSubcurve2::Cubic(curved),
                    },
                ],
                &policy,
            )
            .expect("the exact chord and curved return must close");
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid exact retained-chord region");
            let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
                .expect("valid unary exact-chord context");
            assert!(matches!(
                context.data.carriers[0].geometry,
                RegionCarrierGeometry::AlgebraicChord(_)
            ));
            let regularized = context.build_regularized_region();
            assert!(
                regularized.is_ok(),
                "the exact retained chord must preserve the canonical chord authority: {regularized:?}"
            );
        }
    }

    #[test]
    fn exact_subfragment_of_algebraic_chord_retains_selected_field_witness() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let region = selected_field_algebraic_chord_rectangle(&policy);
            let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
                .expect("valid unary algebraic-chord context");
            let RegionCarrierGeometry::AlgebraicChord(source) = &context.data.carriers[0].geometry
            else {
                panic!("the selected-field bottom edge must stay algebraic");
            };
            let cut = |x: Real| {
                decided(
                    source
                        .parameter_at_certified_point(
                            RationalBezierIntersectionPointEvidence2::Exact(Point2::new(
                                x,
                                Real::zero(),
                            )),
                            &policy,
                        )
                        .expect("exact point on retained support"),
                )
                .expect("the exact point lies strictly on the source chord")
            };
            let exact_subfragment = crate::BezierAlgebraicChord2::from_ordered_parameter_range(
                source,
                &cut(Real::zero()),
                &cut((Real::one() / Real::from(2_u8)).expect("nonzero denominator")),
                &policy,
            )
            .expect("ordered exact subfragment");
            assert!(exact_subfragment.exact_line().is_some());
            let action = context
                .regularized_algebraic_chord_fragment_decision(0, &exact_subfragment)
                .map(|decision| decision.action);
            assert!(
                action.is_ok(),
                "an exact subfragment of an algebraic carrier must retain a selected-field side witness: {action:?}"
            );
        }
    }

    #[test]
    fn curve_trim_retains_selected_field_algebraic_chord_boundaries() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let region = selected_field_algebraic_chord_rectangle(&policy);
            let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
            let source = Curve2::from(
                LineSeg2::try_new(
                    Point2::new(Real::from(-1_i8), half.clone()),
                    Point2::new(Real::one(), half),
                )
                .expect("valid horizontal cutter"),
            );

            let trimmed = source
                .trim_inside_region_with_parameters(&region, &policy)
                .expect("a rational line must trim against selected-field chords");
            assert_eq!(trimmed.certainty, crate::CurveCertainty::Certified);
            let [trimmed] = trimmed.value.as_slice() else {
                panic!("the algebraic rectangle must retain one exact interval");
            };
            assert!(matches!(
                trimmed.fragment(),
                BezierSplitFragment2::AlgebraicEndpointImages { .. }
            ));
            assert!(trimmed.represented_parameter_range().is_none());
            assert_eq!(trimmed.start_boundary_contacts().len(), 1);
            assert_eq!(trimmed.end_boundary_contacts().len(), 1);
            assert_eq!(trimmed.start_boundary_contacts()[0].segment_index(), 3);
            assert_eq!(trimmed.end_boundary_contacts()[0].segment_index(), 1);
            assert!(
                trimmed.start_boundary_contacts()[0]
                    .boundary_parameter()
                    .is_algebraic_chord()
            );
            assert!(
                trimmed.end_boundary_contacts()[0]
                    .boundary_parameter()
                    .is_algebraic_chord()
            );
        }
    }

    #[test]
    fn curve_trim_retains_selected_field_algebraic_chord_overlaps() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let region = selected_field_algebraic_chord_rectangle(&policy);
                let BezierSplitFragment2::AlgebraicChord(bottom) =
                    &region.boundary_loops()[0].fragments()[0]
                else {
                    unreachable!("the selected-field bottom edge is an algebraic chord");
                };
                let (start, end) = if reversed {
                    (Point2::from_values(1, 0), Point2::from_values(-1, 0))
                } else {
                    (Point2::from_values(-1, 0), Point2::from_values(1, 0))
                };
                let source = Curve2::from(
                    LineSeg2::try_new(start, end).expect("valid horizontal overlap source"),
                );

                let trimmed = source
                    .trim_inside_region_with_parameters(&region, &policy)
                    .expect("selected-field boundary overlap must trim exactly");
                assert_eq!(trimmed.certainty, crate::CurveCertainty::Certified);
                let [trimmed] = trimmed.value.as_slice() else {
                    panic!("the selected-field bottom edge must retain one exact interval");
                };
                assert!(matches!(
                    trimmed.fragment(),
                    BezierSplitFragment2::AlgebraicEndpointImages { .. }
                ));
                assert!(trimmed.represented_parameter_range().is_none());
                let start_contact = trimmed
                    .start_boundary_contacts()
                    .iter()
                    .find(|contact| contact.segment_index() == 0)
                    .expect("the overlap start must retain bottom-edge provenance");
                let end_contact = trimmed
                    .end_boundary_contacts()
                    .iter()
                    .find(|contact| contact.segment_index() == 0)
                    .expect("the overlap end must retain bottom-edge provenance");
                let start_parameter = start_contact
                    .boundary_parameter()
                    .as_algebraic_chord()
                    .expect("bottom-edge contact must retain its chord parameter");
                let end_parameter = end_contact
                    .boundary_parameter()
                    .as_algebraic_chord()
                    .expect("bottom-edge contact must retain its chord parameter");
                let (expected_start, expected_end) = if reversed {
                    (bottom.end_parameter(), bottom.start_parameter())
                } else {
                    (bottom.start_parameter(), bottom.end_parameter())
                };
                assert_eq!(
                    start_parameter
                        .cmp_by_refinement(&expected_start, &policy)
                        .unwrap(),
                    Classification::Decided(Ordering::Equal)
                );
                assert_eq!(
                    end_parameter
                        .cmp_by_refinement(&expected_end, &policy)
                        .unwrap(),
                    Classification::Decided(Ordering::Equal)
                );
            }
        }
    }

    #[test]
    fn source_related_algebraic_chord_contact_enters_split_topology() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let third = (Real::one() / Real::from(3_i8)).expect("nonzero denominator");
            let source_curve = BezierSubcurve2::Cubic(CubicBezier2::new(
                Point2::from_values(1, 0),
                Point2::new(Real::one() + &third, third.clone()),
                Point2::new(
                    Real::one() + Real::from(2_i8) * &third,
                    Real::from(2_i8) * &third,
                ),
                Point2::from_values(2, 0),
            ));
            let source_rational =
                RationalBezier2::try_from_subcurve(&source_curve).expect("valid rational source");
            let parameter = sqrt_half_parameter(&policy);
            let source_parameter = BezierParameter2::Algebraic(parameter.clone());
            let materialization = decided(
                source_curve
                    .split_at_parameters(std::slice::from_ref(&source_parameter), &policy)
                    .expect("exact algebraic source split"),
            );
            let source_fragment = materialization.fragments()[0].clone();
            let selected_point =
                exact_contact_point_evidence(&source_rational, &source_parameter, &policy)
                    .expect("exact selected point construction")
                    .expect("selected point evidence");
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    selected_point,
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(0, 0)),
                    &policy,
                )
                .expect("valid algebraic chord"),
            );
            let closure = LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                .expect("valid closure");
            let boundary = CurveRegionBoundaryLoop2::new(
                vec![
                    source_fragment,
                    BezierSplitFragment2::AlgebraicChord(chord),
                    BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                            closure,
                        )),
                    },
                ],
                &policy,
            )
            .expect("the correlated self-crossing loop must close exactly");
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid retained test region");
            let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
                .expect("valid unary Boolean context");
            let pair = context
                .data
                .pairs
                .iter()
                .find(|pair| {
                    matches!(
                        context.data.carriers[pair.first_carrier_index].geometry,
                        RegionCarrierGeometry::AlgebraicChord(_)
                    ) || matches!(
                        context.data.carriers[pair.second_carrier_index].geometry,
                        RegionCarrierGeometry::AlgebraicChord(_)
                    )
                })
                .expect("the source/chord pair must be scheduled");
            let pair_result = context
                .pair_result(pair)
                .expect("source-related pair replay must complete");
            assert!(pair_result.blockers.is_empty());
            assert_eq!(pair_result.contacts.len(), 1);
            assert!(pair_result.contacts[0].is_certified_transverse());
            assert!(
                pair_result.contacts[0]
                    .first_parameter()
                    .is_algebraic_chord()
                    || pair_result.contacts[0]
                        .second_parameter()
                        .is_algebraic_chord()
            );

            let topology = context
                .build_split_topology()
                .expect("the correlated contact must enter the common split topology");
            let chord_index = context
                .data
                .carriers
                .iter()
                .position(|carrier| {
                    matches!(carrier.geometry, RegionCarrierGeometry::AlgebraicChord(_))
                })
                .expect("retained chord carrier");
            assert_eq!(topology.split_fragments[chord_index].len(), 2);
            assert_eq!(
                topology.split_fragments[pair.first_carrier_index].len()
                    + topology.split_fragments[pair.second_carrier_index].len(),
                4
            );
            for split in &topology.split_fragments[chord_index] {
                let BezierSplitFragment2::AlgebraicChord(chord) = &split.fragment else {
                    unreachable!();
                };
                let representative = chord.representative_point(&policy);
                assert!(
                    matches!(representative, Ok(Classification::Decided(_))),
                    "split chord representative: {representative:?}"
                );
                let Classification::Decided(RationalBezierIntersectionPointEvidence2::Algebraic(
                    representative,
                )) = representative.expect("representative construction")
                else {
                    panic!("the split chord representative must remain algebraic");
                };
                let [tangent_x, tangent_y] = chord
                    .tangent_coordinate_signs(&policy)
                    .expect("tangent signs");
                for left in [true, false] {
                    let side = context
                        .algebraic_fragment_side_classification(
                            chord_index,
                            &representative,
                            tangent_x,
                            tangent_y,
                            left,
                        )
                        .map(|(_, location)| location);
                    assert!(side.is_ok(), "split chord side {left}: {side:?}");
                }
                let action = context
                    .regularized_algebraic_chord_fragment_decision(chord_index, chord)
                    .map(|decision| decision.action);
                assert!(action.is_ok(), "split chord action: {action:?}");
            }
            let regularized = context.build_regularized_region();
            assert!(
                regularized.is_ok(),
                "the split algebraic chord must traverse the arrangement: {regularized:?}"
            );
        }
    }

    #[test]
    fn nonadjacent_source_chord_pair_replays_endpoint_and_residual_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let third = (Real::one() / Real::from(3_i8)).expect("nonzero denominator");
            let source_curve = BezierSubcurve2::Cubic(CubicBezier2::new(
                Point2::from_values(1, 0),
                Point2::new(Real::one() + &third, third.clone()),
                Point2::new(
                    Real::one() + Real::from(2_i8) * &third,
                    Real::from(2_i8) * &third,
                ),
                Point2::from_values(2, 0),
            ));
            let source_rational =
                RationalBezier2::try_from_subcurve(&source_curve).expect("valid rational source");
            let parameter = sqrt_half_parameter(&policy);
            let source_parameter = BezierParameter2::Algebraic(parameter);
            let source_fragment = decided(
                source_curve
                    .split_at_parameters(std::slice::from_ref(&source_parameter), &policy)
                    .expect("exact algebraic source split"),
            )
            .fragments()[0]
                .clone();
            let selected_point =
                exact_contact_point_evidence(&source_rational, &source_parameter, &policy)
                    .expect("exact selected point construction")
                    .expect("selected point evidence");
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    selected_point,
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(0, 0)),
                    &policy,
                )
                .expect("valid algebraic chord"),
            );
            let chord_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    source_fragment,
                    BezierSplitFragment2::AlgebraicChord(chord),
                    BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                                .expect("valid chord-loop closure"),
                        )),
                    },
                ],
                &policy,
            )
            .expect("the retained chord loop must close");
            let source_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: source_curve,
                    },
                    BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                            LineSeg2::try_new(Point2::from_values(2, 0), Point2::from_values(1, 0))
                                .expect("valid source-loop closure"),
                        )),
                    },
                ],
                &policy,
            )
            .expect("the complete source loop must close");
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![chord_loop, source_loop],
                vec![CurveRegionLoopRole::Material; 2],
                vec![FillRule::NonZero; 2],
                vec![crate::CurveBoundaryInteriorSide2::Left; 2],
            )
            .expect("valid multi-loop retained test region");
            let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
                .expect("valid unary Boolean context");
            let chord_index = context
                .data
                .carriers
                .iter()
                .position(|carrier| {
                    carrier.loop_index == 0
                        && matches!(carrier.geometry, RegionCarrierGeometry::AlgebraicChord(_))
                })
                .expect("retained chord carrier");
            let source_index = context
                .data
                .carriers
                .iter()
                .position(|carrier| carrier.loop_index == 1 && carrier.fragment_index == 0)
                .expect("nonadjacent source carrier");
            let pair = context
                .data
                .pairs
                .iter()
                .find(|pair| {
                    (pair.first_carrier_index == chord_index
                        && pair.second_carrier_index == source_index)
                        || (pair.first_carrier_index == source_index
                            && pair.second_carrier_index == chord_index)
                })
                .expect("the overlapping chord/source bounds must schedule the pair");
            assert!(!context.authored_carriers_are_adjacent(pair));
            let result = context
                .pair_result(pair)
                .expect("nonadjacent general source/chord replay must complete");
            assert!(
                result.blockers.is_empty(),
                "nonadjacent general source/chord result: {result:?}"
            );
            assert_eq!(result.contacts.len(), 2);
            assert!(
                result
                    .contacts
                    .iter()
                    .all(RegionPairContactEvidence::is_certified_transverse)
            );
        }
    }

    #[test]
    fn independent_field_algebraic_chord_uses_general_boolean_pair_engine() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first_parameter = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
            let second_parameter = BezierParameter2::Algebraic(sqrt_third_parameter(&policy));
            let x_axis = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                    .expect("valid x axis"),
            ));
            let y_axis = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(0, 1))
                    .expect("valid y axis"),
            ));
            let x_rational =
                RationalBezier2::try_from_subcurve(&x_axis).expect("valid rational x axis");
            let y_rational =
                RationalBezier2::try_from_subcurve(&y_axis).expect("valid rational y axis");
            let start = exact_contact_point_evidence(&x_rational, &first_parameter, &policy)
                .expect("exact first endpoint")
                .expect("first endpoint evidence");
            let end = exact_contact_point_evidence(&y_rational, &second_parameter, &policy)
                .expect("exact second endpoint")
                .expect("second endpoint evidence");
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(start, end, &policy)
                    .expect("valid independent-field chord"),
            );
            let x_fragment = decided(
                x_axis
                    .split_at_parameters(std::slice::from_ref(&first_parameter), &policy)
                    .expect("exact x-axis split"),
            )
            .fragments()[0]
                .clone();
            let y_fragment = decided(
                y_axis
                    .split_at_parameters(std::slice::from_ref(&second_parameter), &policy)
                    .expect("exact y-axis split"),
            )
            .fragments()[0]
                .reversed()
                .expect("exact y-axis reversal");
            let chord_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::AlgebraicChord(chord),
                    y_fragment,
                    x_fragment,
                ],
                &policy,
            )
            .expect("independent-field chord loop must close");

            let diagonal = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 1))
                    .expect("valid diagonal"),
            ));
            let materialized = |curve| BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve,
            };
            let source_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    materialized(diagonal),
                    materialized(BezierSubcurve2::Quadratic(
                        QuadraticBezier2::from_line_segment(
                            LineSeg2::try_new(
                                Point2::from_values(1, 1),
                                Point2::from_values(-1, 1),
                            )
                            .expect("valid source-loop top"),
                        ),
                    )),
                    materialized(BezierSubcurve2::Quadratic(
                        QuadraticBezier2::from_line_segment(
                            LineSeg2::try_new(
                                Point2::from_values(-1, 1),
                                Point2::from_values(0, 0),
                            )
                            .expect("valid source-loop closure"),
                        ),
                    )),
                ],
                &policy,
            )
            .expect("source loop must close");
            let chord_region = CurveRegion2::try_new_with_loop_topology(
                vec![chord_loop],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid independent-field chord region");
            let source_region = CurveRegion2::try_new_with_loop_topology(
                vec![source_loop],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid diagonal source region");
            let context =
                CurveRegionBooleanContext::try_new(&chord_region, &source_region, &policy)
                    .expect("valid independent-field Boolean context");
            let chord_index = context
                .data
                .carriers
                .iter()
                .position(|carrier| {
                    carrier.operand == CurveRegionBooleanOperand2::First
                        && matches!(carrier.geometry, RegionCarrierGeometry::AlgebraicChord(_))
                })
                .expect("retained independent-field chord carrier");
            let source_index = context
                .data
                .carriers
                .iter()
                .position(|carrier| {
                    carrier.operand == CurveRegionBooleanOperand2::Second
                        && carrier.fragment_index == 0
                })
                .expect("nonadjacent diagonal carrier");
            let pair = context
                .data
                .pairs
                .iter()
                .find(|pair| {
                    (pair.first_carrier_index == chord_index
                        && pair.second_carrier_index == source_index)
                        || (pair.first_carrier_index == source_index
                            && pair.second_carrier_index == chord_index)
                })
                .expect("overlapping chord/diagonal bounds must schedule the pair");
            assert!(!context.authored_carriers_are_adjacent(pair));
            let result = context
                .pair_result(pair)
                .expect("general independent-field pair replay must complete");
            assert!(
                result.blockers.is_empty(),
                "independent-field pair result: {result:?}"
            );
            assert_eq!(result.contacts.len(), 1);
            assert!(result.contacts[0].is_certified_transverse());
            let topology = context.build_split_topology();
            assert!(
                topology.is_ok(),
                "independent-field contact must enter split topology: {topology:?}"
            );
            let booleans = context.build_boolean_regions();
            assert!(
                booleans.is_ok(),
                "independent-field contact must traverse all Booleans: {booleans:?}"
            );
        }
    }

    #[test]
    fn independent_field_collinear_chord_overlap_enters_all_boolean_topology() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let half_parameter = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
            let third_parameter = BezierParameter2::Algebraic(sqrt_third_parameter(&policy));
            let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
            let third = (Real::one() / Real::from(3_i8)).expect("nonzero denominator");
            let apex = Point2::new(Real::zero(), -third.clone());
            let half_source = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                apex.clone(),
                Point2::new(half.clone(), -third.clone()),
                Point2::new(Real::one(), third.clone()),
            ));
            let third_source = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                apex,
                Point2::new(half, -third.clone()),
                Point2::new(Real::one(), Real::from(2_i8) * &third),
            ));
            let half_rational =
                RationalBezier2::try_from_subcurve(&half_source).expect("valid half source");
            let third_rational =
                RationalBezier2::try_from_subcurve(&third_source).expect("valid third source");
            let half_point = exact_contact_point_evidence(&half_rational, &half_parameter, &policy)
                .expect("exact half endpoint")
                .expect("half endpoint evidence");
            let third_point =
                exact_contact_point_evidence(&third_rational, &third_parameter, &policy)
                    .expect("exact third endpoint")
                    .expect("third endpoint evidence");
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    half_point.clone(),
                    third_point.clone(),
                    &policy,
                )
                .expect("valid independent-field horizontal chord"),
            );
            let horizontal_source = rational_line(0, 1);
            let chord_geometry = RegionCarrierGeometry::AlgebraicChord(chord.clone());
            let source_geometry =
                RegionCarrierGeometry::Bezier(BezierSubcurve2::Rational(horizontal_source.clone()));
            let source_low =
                (Real::from(3_i8) / Real::from(5_i8)).expect("nonzero source-range denominator");
            let source_high =
                (Real::from(2_i8) / Real::from(3_i8)).expect("nonzero source-range denominator");
            let carriers = vec![
                RegionCarrier {
                    operand: CurveRegionBooleanOperand2::First,
                    loop_index: 0,
                    fragment_index: 0,
                    family: chord_geometry.family(),
                    geometry: chord_geometry,
                    start: CurveRegionParameter2::from_algebraic_chord(chord.start_parameter()),
                    end: CurveRegionParameter2::from_algebraic_chord(chord.end_parameter()),
                    reversed: false,
                    filled_side_is_left: true,
                    selected_fiber_endpoint_points: None,
                    image_is_injective: OnceLock::new(),
                    bounds: OnceLock::new(),
                },
                RegionCarrier {
                    operand: CurveRegionBooleanOperand2::Second,
                    loop_index: 0,
                    fragment_index: 0,
                    family: source_geometry.family(),
                    geometry: source_geometry,
                    start: CurveRegionParameter2::from_bezier(BezierParameter2::Exact(
                        source_low.clone(),
                    )),
                    end: CurveRegionParameter2::from_bezier(BezierParameter2::Exact(
                        source_high.clone(),
                    )),
                    reversed: false,
                    filled_side_is_left: true,
                    selected_fiber_endpoint_points: None,
                    image_is_injective: OnceLock::new(),
                    bounds: OnceLock::new(),
                },
            ];
            let empty_first = CurveRegion2::empty();
            let empty_second = CurveRegion2::empty();
            let clipping_context = CurveRegionBooleanContext {
                data: CurveRegionBooleanContextData {
                    first: &empty_first,
                    second: &empty_second,
                    policy,
                    carriers,
                    first_carrier_count: 1,
                    authored_carrier_pair_count: 1,
                    pairs: Vec::new(),
                    bezier_self_intersections: Vec::new(),
                    parallel_self_intersections: Vec::new(),
                    strict_line_image_only: OnceLock::new(),
                },
            };
            let clipping_pair = RegionCarrierPair {
                first_carrier_index: 0,
                second_carrier_index: 1,
                context: RegionCarrierPairContext::AlgebraicChordPair,
            };
            let clipping_result = clipping_context
                .pair_result(&clipping_pair)
                .expect("full collinear overlap must complete before clipping");
            let [raw_overlap] = clipping_result.overlaps.as_slice() else {
                panic!("expected one raw overlap, got {clipping_result:?}");
            };
            let (clipped_chord_range, clipped_source_range) = clipping_context
                .clipped_overlap_ranges(&clipping_pair, raw_overlap)
                .expect("authored source subrange must clip exactly")
                .expect("the authored source subrange lies inside the chord");
            assert!(clipped_chord_range.start().is_algebraic_chord());
            assert_eq!(
                clipped_source_range
                    .start()
                    .as_bezier_parameter()
                    .expect("Bezier source range")
                    .cmp_by_refinement(&BezierParameter2::Exact(source_high.clone()), &policy,)
                    .expect("exact source-range order"),
                Classification::Decided(Ordering::Equal)
            );
            assert_eq!(
                clipped_source_range
                    .end()
                    .as_bezier_parameter()
                    .expect("Bezier source range")
                    .cmp_by_refinement(&BezierParameter2::Exact(source_low.clone()), &policy)
                    .expect("exact source-range order"),
                Classification::Decided(Ordering::Equal)
            );
            let half_fragment = decided(
                half_source
                    .split_at_parameters(std::slice::from_ref(&half_parameter), &policy)
                    .expect("exact half-source split"),
            )
            .fragments()[0]
                .clone();
            let third_fragment = decided(
                third_source
                    .split_at_parameters(std::slice::from_ref(&third_parameter), &policy)
                    .expect("exact third-source split"),
            )
            .fragments()[0]
                .reversed()
                .expect("exact third-source reversal");
            let chord_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::AlgebraicChord(chord),
                    third_fragment,
                    half_fragment,
                ],
                &policy,
            )
            .expect("independent-field chord loop must close");
            let chord_region = CurveRegion2::try_new_with_loop_topology(
                vec![chord_loop],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid independent-field chord region");

            let materialized_line =
                |start_x, start_y, end_x, end_y| BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                        LineSeg2::try_new(
                            Point2::from_values(start_x, start_y),
                            Point2::from_values(end_x, end_y),
                        )
                        .expect("valid rectangle edge"),
                    )),
                };
            let source_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    materialized_line(0, 0, 1, 0),
                    materialized_line(1, 0, 1, 1),
                    materialized_line(1, 1, 0, 1),
                    materialized_line(0, 1, 0, 0),
                ],
                &policy,
            )
            .expect("source rectangle must close");
            let source_region = CurveRegion2::try_new_with_loop_topology(
                vec![source_loop],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid source rectangle");

            let intersections = chord_region
                .intersect_region(&source_region, &policy)
                .expect("collinear chord/source intersection must complete")
                .into_value();
            assert!(intersections.is_complete(), "{intersections:?}");
            assert_eq!(intersections.overlaps().len(), 1, "{intersections:?}");
            assert_eq!(
                intersections.overlaps()[0].orientation(),
                RationalBezierOverlapOrientation2::Reversed
            );
            assert!(
                intersections.overlaps()[0]
                    .first_range()
                    .start()
                    .is_algebraic_chord()
            );

            let booleans = chord_region.boolean_regions(&source_region, &policy);
            assert!(
                booleans.is_ok(),
                "collinear chord overlap must enter all four Booleans: {booleans:?}"
            );
            let booleans = booleans.expect("complete collinear Booleans").into_value();
            assert!(booleans.intersection().is_empty());
            assert!(!booleans.union().is_empty());
            assert!(!booleans.difference().is_empty());
            assert!(!booleans.xor().is_empty());
        }
    }

    #[test]
    fn algebraic_chord_pair_overlap_enters_region_intersection_evidence() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let horizontal = rational_line(0, 1);
            let first_parameter = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
            let second_parameter = BezierParameter2::Algebraic(sqrt_third_parameter(&policy));
            let first_point = exact_contact_point_evidence(&horizontal, &first_parameter, &policy)
                .expect("exact first endpoint")
                .expect("first endpoint evidence");
            let second_point =
                exact_contact_point_evidence(&horizontal, &second_parameter, &policy)
                    .expect("exact second endpoint")
                    .expect("second endpoint evidence");
            let first = decided(
                crate::BezierAlgebraicChord2::try_new(first_point, second_point, &policy)
                    .expect("valid independent-field chord"),
            );
            let second = decided(
                crate::BezierAlgebraicChord2::try_new(
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(1, 0)),
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(0, 0)),
                    &policy,
                )
                .expect("valid represented containing chord"),
            );
            let carrier = |operand, chord: crate::BezierAlgebraicChord2| RegionCarrier {
                operand,
                loop_index: 0,
                fragment_index: 0,
                family: CurveFamily2::Line,
                start: CurveRegionParameter2::from_algebraic_chord(chord.start_parameter()),
                end: CurveRegionParameter2::from_algebraic_chord(chord.end_parameter()),
                geometry: RegionCarrierGeometry::AlgebraicChord(chord),
                reversed: false,
                filled_side_is_left: true,
                selected_fiber_endpoint_points: None,
                image_is_injective: OnceLock::new(),
                bounds: OnceLock::new(),
            };
            let empty_first = CurveRegion2::empty();
            let empty_second = CurveRegion2::empty();
            let context = CurveRegionBooleanContext {
                data: CurveRegionBooleanContextData {
                    first: &empty_first,
                    second: &empty_second,
                    policy,
                    carriers: vec![
                        carrier(CurveRegionBooleanOperand2::First, first),
                        carrier(CurveRegionBooleanOperand2::Second, second),
                    ],
                    first_carrier_count: 1,
                    authored_carrier_pair_count: 1,
                    pairs: vec![RegionCarrierPair {
                        first_carrier_index: 0,
                        second_carrier_index: 1,
                        context: RegionCarrierPairContext::AlgebraicChordPair,
                    }],
                    bezier_self_intersections: Vec::new(),
                    parallel_self_intersections: Vec::new(),
                    strict_line_image_only: OnceLock::new(),
                },
            };
            let pair_result = context
                .pair_result(&context.data.pairs[0])
                .expect("algebraic chord pair overlap must complete");
            assert!(pair_result.blockers.is_empty(), "{pair_result:?}");
            assert!(pair_result.contacts.is_empty(), "{pair_result:?}");
            let [overlap] = pair_result.overlaps.as_slice() else {
                panic!("expected one algebraic chord overlap: {pair_result:?}");
            };
            assert_eq!(overlap.orientation, RationalBezierOverlapOrientation2::Same);
            assert!(overlap.first_range.start().is_algebraic_chord());
            assert!(overlap.second_range.start().is_algebraic_chord());

            let evidence = context
                .build_intersection_evidence()
                .expect("algebraic chord overlap must enter region evidence");
            assert!(evidence.is_complete(), "{evidence:?}");
            assert!(evidence.contacts().is_empty(), "{evidence:?}");
            assert_eq!(evidence.overlaps().len(), 1, "{evidence:?}");
        }
    }

    #[test]
    fn algebraic_chord_exact_linear_bezier_pair_replays_all_line_relations() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::new(
                        Real::from(2_i8).sqrt().unwrap(),
                        Real::zero(),
                    )),
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(4, 0)),
                    &policy,
                )
                .expect("valid exact-field chord"),
            );
            let empty_first = CurveRegion2::empty();
            let empty_second = CurveRegion2::empty();
            let evaluate = |line: LineSeg2| {
                let chord_geometry = RegionCarrierGeometry::AlgebraicChord(chord.clone());
                let curve_geometry = RegionCarrierGeometry::Bezier(BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(line),
                ));
                let context = CurveRegionBooleanContext {
                    data: CurveRegionBooleanContextData {
                        first: &empty_first,
                        second: &empty_second,
                        policy,
                        carriers: vec![
                            RegionCarrier {
                                operand: CurveRegionBooleanOperand2::First,
                                loop_index: 0,
                                fragment_index: 0,
                                family: chord_geometry.family(),
                                geometry: chord_geometry,
                                start: CurveRegionParameter2::from_algebraic_chord(
                                    chord.start_parameter(),
                                ),
                                end: CurveRegionParameter2::from_algebraic_chord(
                                    chord.end_parameter(),
                                ),
                                reversed: false,
                                filled_side_is_left: true,
                                selected_fiber_endpoint_points: None,
                                image_is_injective: OnceLock::new(),
                                bounds: OnceLock::new(),
                            },
                            RegionCarrier {
                                operand: CurveRegionBooleanOperand2::Second,
                                loop_index: 0,
                                fragment_index: 0,
                                family: curve_geometry.family(),
                                geometry: curve_geometry,
                                start: CurveRegionParameter2::from_bezier(BezierParameter2::Exact(
                                    Real::zero(),
                                )),
                                end: CurveRegionParameter2::from_bezier(BezierParameter2::Exact(
                                    Real::one(),
                                )),
                                reversed: false,
                                filled_side_is_left: true,
                                selected_fiber_endpoint_points: None,
                                image_is_injective: OnceLock::new(),
                                bounds: OnceLock::new(),
                            },
                        ],
                        first_carrier_count: 1,
                        authored_carrier_pair_count: 1,
                        pairs: vec![RegionCarrierPair {
                            first_carrier_index: 0,
                            second_carrier_index: 1,
                            context: RegionCarrierPairContext::AlgebraicChordPair,
                        }],
                        bezier_self_intersections: Vec::new(),
                        parallel_self_intersections: Vec::new(),
                        strict_line_image_only: OnceLock::new(),
                    },
                };
                context
                    .pair_result(&context.data.pairs[0])
                    .expect("the direct chord/linear-Bezier relation must complete")
            };

            for (line, orientation) in [
                (
                    LineSeg2::try_new(Point2::from_values(2, 0), Point2::from_values(5, 0))
                        .unwrap(),
                    RationalBezierOverlapOrientation2::Same,
                ),
                (
                    LineSeg2::try_new(Point2::from_values(5, 0), Point2::from_values(2, 0))
                        .unwrap(),
                    RationalBezierOverlapOrientation2::Reversed,
                ),
            ] {
                let result = evaluate(line);
                assert!(result.blockers.is_empty(), "{result:?}");
                assert!(result.contacts.is_empty(), "{result:?}");
                let [overlap] = result.overlaps.as_slice() else {
                    panic!("expected one exact line overlap: {result:?}");
                };
                assert_eq!(overlap.orientation, orientation);
                assert!(overlap.first_range.start().is_algebraic_chord());
                assert!(overlap.second_range.start().is_exact());
            }

            let crossing = evaluate(
                LineSeg2::try_new(Point2::from_values(3, -1), Point2::from_values(3, 1)).unwrap(),
            );
            assert!(crossing.blockers.is_empty(), "{crossing:?}");
            assert!(crossing.overlaps.is_empty(), "{crossing:?}");
            let [contact] = crossing.contacts.as_slice() else {
                panic!("expected one exact transverse contact: {crossing:?}");
            };
            assert!(contact.certified_transverse);
            assert_eq!(contact.tangent_cross_sign, Some(RealSign::Positive));
            assert!(contact.first_parameter.is_algebraic_chord());
            assert!(contact.second_parameter.is_exact());

            let disjoint = evaluate(
                LineSeg2::try_new(Point2::from_values(2, 1), Point2::from_values(5, 1)).unwrap(),
            );
            assert!(disjoint.contacts.is_empty(), "{disjoint:?}");
            assert!(disjoint.overlaps.is_empty(), "{disjoint:?}");
            assert!(disjoint.blockers.is_empty(), "{disjoint:?}");
        }
    }

    #[test]
    fn authored_parallel_support_contact_classifies_perpendicular_and_tangent_joins() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                    .expect("valid horizontal source"),
            );
            let parallel = BezierParallel2::from_source(
                crate::BezierParallelSource2::Quadratic(source),
                Real::zero(),
            );
            let analytic = BezierSplitFragment2::AnalyticParallel(
                BezierParallelFragment2::from_certified_range(
                    parallel.clone(),
                    BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    false,
                ),
            );
            let line = |start_x, start_y, end_x, end_y| BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(
                        Point2::from_values(start_x, start_y),
                        Point2::from_values(end_x, end_y),
                    )
                    .expect("valid rectangle edge"),
                )),
            };
            let boundary = CurveRegionBoundaryLoop2::new(
                vec![
                    analytic,
                    line(1, 0, 1, 1),
                    line(1, 1, 0, 1),
                    line(0, 1, 0, 0),
                ],
                &policy,
            )
            .expect("analytic rectangle must close");
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid analytic rectangle");
            let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
                .expect("valid unary context");
            let parallel_index = context
                .data
                .carriers
                .iter()
                .position(|carrier| {
                    matches!(carrier.geometry, RegionCarrierGeometry::AnalyticParallel(_))
                })
                .expect("the analytic carrier must be retained");
            let line_index = context
                .data
                .carriers
                .iter()
                .position(|carrier| carrier.fragment_index == 1)
                .expect("the following line carrier must be retained");
            let pair = RegionCarrierPair {
                first_carrier_index: parallel_index,
                second_carrier_index: line_index,
                context: RegionCarrierPairContext::ParallelPair,
            };
            let parallel_carrier = &context.data.carriers[parallel_index];
            let range = CurveRegionParameterRange2::new_validated(
                parallel_carrier.start.clone(),
                parallel_carrier.end.clone(),
            );
            let perpendicular = context
                .authored_parallel_support_contact(
                    &pair,
                    &parallel,
                    parallel_index,
                    &Real::zero(),
                    &Real::one(),
                    Some(&range),
                )
                .expect("the perpendicular relation must decide")
                .expect("the perpendicular authored contact must be retained");
            assert_eq!(perpendicular, (Real::one(), RealSign::Negative));
            let tangent = context
                .authored_parallel_support_contact(
                    &pair,
                    &parallel,
                    parallel_index,
                    &Real::one(),
                    &Real::zero(),
                    Some(&range),
                )
                .expect("the tangent relation must decide")
                .expect("the tangent authored contact must be retained");
            assert_eq!(tangent, (Real::one(), RealSign::Zero));
        }
    }

    #[test]
    fn algebraic_chord_analytic_parallel_pair_replays_contacts_and_overlap() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let empty_first = CurveRegion2::empty();
            let empty_second = CurveRegion2::empty();
            let evaluate = |chord: crate::BezierAlgebraicChord2, parallel: BezierParallel2| {
                let chord_geometry = RegionCarrierGeometry::AlgebraicChord(chord.clone());
                let parallel_geometry = RegionCarrierGeometry::AnalyticParallel(parallel);
                let context = CurveRegionBooleanContext {
                    data: CurveRegionBooleanContextData {
                        first: &empty_first,
                        second: &empty_second,
                        policy,
                        carriers: vec![
                            RegionCarrier {
                                operand: CurveRegionBooleanOperand2::First,
                                loop_index: 0,
                                fragment_index: 0,
                                family: chord_geometry.family(),
                                geometry: chord_geometry,
                                start: CurveRegionParameter2::from_algebraic_chord(
                                    chord.start_parameter(),
                                ),
                                end: CurveRegionParameter2::from_algebraic_chord(
                                    chord.end_parameter(),
                                ),
                                reversed: false,
                                filled_side_is_left: true,
                                selected_fiber_endpoint_points: None,
                                image_is_injective: OnceLock::new(),
                                bounds: OnceLock::new(),
                            },
                            RegionCarrier {
                                operand: CurveRegionBooleanOperand2::Second,
                                loop_index: 0,
                                fragment_index: 0,
                                family: parallel_geometry.family(),
                                geometry: parallel_geometry,
                                start: CurveRegionParameter2::from_bezier(BezierParameter2::Exact(
                                    Real::zero(),
                                )),
                                end: CurveRegionParameter2::from_bezier(BezierParameter2::Exact(
                                    Real::one(),
                                )),
                                reversed: false,
                                filled_side_is_left: true,
                                selected_fiber_endpoint_points: None,
                                image_is_injective: OnceLock::new(),
                                bounds: OnceLock::new(),
                            },
                        ],
                        first_carrier_count: 1,
                        authored_carrier_pair_count: 1,
                        pairs: vec![RegionCarrierPair {
                            first_carrier_index: 0,
                            second_carrier_index: 1,
                            context: RegionCarrierPairContext::AlgebraicChordPair,
                        }],
                        bezier_self_intersections: Vec::new(),
                        parallel_self_intersections: Vec::new(),
                        strict_line_image_only: OnceLock::new(),
                    },
                };
                context
                    .pair_result(&context.data.pairs[0])
                    .expect("the chord/analytic-parallel relation must complete")
            };

            let crossing_chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(-3, 1)),
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(3, 1)),
                    &policy,
                )
                .unwrap(),
            );
            let parabola = BezierParallel2::from_source(
                crate::BezierParallelSource2::Quadratic(QuadraticBezier2::new(
                    Point2::from_values(-2, 0),
                    Point2::from_values(0, 4),
                    Point2::from_values(2, 0),
                )),
                Real::zero(),
            );
            let crossings = evaluate(crossing_chord, parabola);
            assert!(crossings.blockers.is_empty(), "{crossings:?}");
            assert!(crossings.overlaps.is_empty(), "{crossings:?}");
            assert_eq!(crossings.contacts.len(), 2, "{crossings:?}");
            assert!(crossings.contacts.iter().all(|contact| {
                contact.first_parameter.is_algebraic_chord()
                    && contact.second_parameter.as_bezier_parameter().is_some()
                    && contact.certified_transverse
            }));

            let overlap_chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(-1, 0)),
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(1, 0)),
                    &policy,
                )
                .unwrap(),
            );
            let line_parallel = BezierParallel2::from_source(
                crate::BezierParallelSource2::Quadratic(QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(Point2::from_values(-2, 0), Point2::from_values(2, 0))
                        .unwrap(),
                )),
                Real::zero(),
            );
            let overlap = evaluate(overlap_chord, line_parallel);
            assert!(overlap.blockers.is_empty(), "{overlap:?}");
            assert!(overlap.contacts.is_empty(), "{overlap:?}");
            let [overlap] = overlap.overlaps.as_slice() else {
                panic!("expected one chord/parallel overlap: {overlap:?}");
            };
            assert_eq!(overlap.orientation, RationalBezierOverlapOrientation2::Same);
            assert!(overlap.first_range.start().is_algebraic_chord());
            assert!(overlap.second_range.start().as_bezier_parameter().is_some());
        }
    }

    #[test]
    fn strict_interior_algebraic_chord_pair_contact_splits_both_carriers() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let fraction = |numerator: i8, denominator: i8| {
                (Real::from(numerator) / Real::from(denominator)).expect("nonzero test denominator")
            };
            let sqrt_parameter = |numerator: i8, denominator: i8| {
                let polynomial = decided(
                    crate::BezierParameterPolynomial::try_new_power_basis(
                        vec![
                            Real::from(-numerator),
                            Real::zero(),
                            Real::from(denominator),
                        ],
                        &policy,
                    )
                    .expect("valid square-root polynomial"),
                );
                let interval = decided(
                    crate::BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy)
                        .expect("valid unit interval"),
                );
                BezierParameter2::Algebraic(decided(
                    BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
                        .expect("isolated square-root parameter"),
                ))
            };
            let horizontal = rational_line(0, 1);
            let first_start = exact_contact_point_evidence(
                &horizontal,
                &BezierParameter2::Algebraic(sqrt_half_parameter(&policy)),
                &policy,
            )
            .expect("exact first start")
            .expect("first start evidence");
            let first_end = exact_contact_point_evidence(
                &horizontal,
                &BezierParameter2::Algebraic(sqrt_third_parameter(&policy)),
                &policy,
            )
            .expect("exact first end")
            .expect("first end evidence");
            let vertical = RationalBezier2::try_new(
                vec![
                    Point2::new(fraction(5, 8), Real::from(-1_i8)),
                    Point2::new(fraction(5, 8), Real::one()),
                ],
                vec![Real::one(); 2],
            )
            .expect("valid vertical source");
            let second_start_parameter = sqrt_parameter(2, 5);
            let second_end_parameter = sqrt_parameter(1, 5);
            let second_start =
                exact_contact_point_evidence(&vertical, &second_start_parameter, &policy)
                    .expect("exact second start")
                    .expect("second start evidence");
            let second_end =
                exact_contact_point_evidence(&vertical, &second_end_parameter, &policy)
                    .expect("exact second end")
                    .expect("second end evidence");
            let first = decided(
                crate::BezierAlgebraicChord2::try_new(
                    first_start.clone(),
                    first_end.clone(),
                    &policy,
                )
                .expect("valid horizontal chord"),
            );
            let second = decided(
                crate::BezierAlgebraicChord2::try_new(
                    second_start.clone(),
                    second_end.clone(),
                    &policy,
                )
                .expect("valid vertical chord"),
            );
            let empty_first = CurveRegion2::empty();
            let empty_second = CurveRegion2::empty();
            let pair = RegionCarrierPair {
                first_carrier_index: 0,
                second_carrier_index: 1,
                context: RegionCarrierPairContext::AlgebraicChordPair,
            };
            let context = CurveRegionBooleanContext {
                data: CurveRegionBooleanContextData {
                    first: &empty_first,
                    second: &empty_second,
                    policy,
                    carriers: vec![
                        algebraic_chord_carrier(CurveRegionBooleanOperand2::First, first.clone()),
                        algebraic_chord_carrier(CurveRegionBooleanOperand2::Second, second.clone()),
                    ],
                    first_carrier_count: 1,
                    authored_carrier_pair_count: 1,
                    pairs: Vec::new(),
                    bezier_self_intersections: Vec::new(),
                    parallel_self_intersections: Vec::new(),
                    strict_line_image_only: OnceLock::new(),
                },
            };
            let result = context
                .pair_result(&pair)
                .expect("strict interior chord pair must complete");
            assert!(result.blockers.is_empty(), "{result:?}");
            let [contact] = result.contacts.as_slice() else {
                panic!("expected one strict interior chord contact: {result:?}");
            };
            assert!(contact.is_certified_transverse());
            assert!(matches!(
                contact.point(),
                Some(RationalBezierIntersectionPointEvidence2::AlgebraicChordPair(_))
            ));
            for (carrier, chord, parameter) in [
                (&context.data.carriers[0], &first, contact.first_parameter()),
                (
                    &context.data.carriers[1],
                    &second,
                    contact.second_parameter(),
                ),
            ] {
                let events = vec![
                    CarrierEvent {
                        parameter: carrier.start.clone(),
                        topology_vertex: Some(0),
                    },
                    CarrierEvent {
                        parameter: parameter.clone(),
                        topology_vertex: Some(2),
                    },
                    CarrierEvent {
                        parameter: carrier.end.clone(),
                        topology_vertex: Some(1),
                    },
                ];
                let splits = split_algebraic_chord_carrier(carrier, chord, &events, &policy)
                    .expect("correlated interior contact must split the chord");
                assert_eq!(splits.len(), 2);
                assert_eq!(splits[0].end_topology_vertex, Some(2));
                assert_eq!(splits[1].start_topology_vertex, Some(2));
                assert!(splits.iter().all(|split| matches!(
                    split.fragment,
                    BezierSplitFragment2::AlgebraicChord(_)
                )));
            }

            let first_apex = RationalBezierIntersectionPointEvidence2::Exact(Point2::new(
                fraction(16, 25),
                Real::from(-1_i8),
            ));
            let second_apex = RationalBezierIntersectionPointEvidence2::Exact(Point2::new(
                Real::one(),
                fraction(1, 20),
            ));
            let close = |start, end| {
                BezierSplitFragment2::AlgebraicChord(decided(
                    crate::BezierAlgebraicChord2::try_new(start, end, &policy)
                        .expect("valid triangle closure chord"),
                ))
            };
            let first_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::AlgebraicChord(first),
                    close(first_end, first_apex.clone()),
                    close(first_apex, first_start),
                ],
                &policy,
            )
            .expect("first algebraic chord triangle must close");
            let second_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::AlgebraicChord(second),
                    close(second_end, second_apex.clone()),
                    close(second_apex, second_start),
                ],
                &policy,
            )
            .expect("second algebraic chord triangle must close");
            let region = |boundary| {
                CurveRegion2::try_new_with_loop_topology(
                    vec![boundary],
                    vec![CurveRegionLoopRole::Material],
                    vec![FillRule::NonZero],
                    vec![crate::CurveBoundaryInteriorSide2::Left],
                )
                .expect("valid algebraic chord triangle")
            };
            let first_region = region(first_loop);
            let second_region = region(second_loop);
            let intersections = first_region
                .intersect_region(&second_region, &policy)
                .expect("public strict interior chord intersection must complete");
            assert_eq!(intersections.certainty, crate::CurveCertainty::Certified);
            assert!(intersections.value.is_complete(), "{intersections:?}");
            assert!(
                intersections
                    .value
                    .contacts()
                    .iter()
                    .any(|contact| matches!(
                        contact.point(),
                        Some(RationalBezierIntersectionPointEvidence2::AlgebraicChordPair(_))
                    )),
                "{intersections:?}"
            );
            let boolean_context =
                CurveRegionBooleanContext::try_new(&first_region, &second_region, &policy)
                    .expect("valid strict-interior Boolean context");
            let mut retained_pair_points = Vec::new();
            for pair in &boolean_context.data.pairs {
                let pair_result = boolean_context.pair_result(pair).unwrap();
                retained_pair_points.extend(
                    pair_result
                        .contacts
                        .iter()
                        .filter_map(|contact| contact.point().cloned()),
                );
            }
            assert_eq!(retained_pair_points.len(), 2);
            assert_eq!(
                retained_pair_points[0].same_point(&retained_pair_points[1], &policy),
                Classification::Decided(false),
                "distinct correlated contacts need a certified spatial separation"
            );
            let split_topology = boolean_context.build_split_topology();
            assert!(
                split_topology.is_ok(),
                "strict interior chord contact must build split topology: {split_topology:?}"
            );
            let built = boolean_context.build_boolean_regions();
            assert!(
                built.is_ok(),
                "strict interior chord contact must build Boolean regions: {built:?}"
            );
            let booleans = first_region.boolean_regions(&second_region, &policy);
            assert!(
                booleans.is_ok(),
                "strict interior algebraic chord crossing must traverse all Booleans: {booleans:?}"
            );
            let booleans = booleans
                .expect("complete strict-interior Boolean batch")
                .into_value();
            for (name, result) in [
                ("union", booleans.union()),
                ("intersection", booleans.intersection()),
                ("difference", booleans.difference()),
                ("xor", booleans.xor()),
            ] {
                let replay_context = CurveRegionBooleanContext::try_new_unary(result, &policy)
                    .expect("valid correlated-output arrangement context");
                for pair in &replay_context.data.pairs {
                    let pair_result = replay_context
                        .pair_result(pair)
                        .expect("correlated-output carrier pair replay");
                    assert!(
                        pair_result.blockers.is_empty(),
                        "{name} retained pair {pair:?} must replay exactly: {pair_result:?}"
                    );
                }
                let replay_topology = replay_context.build_split_topology();
                assert!(
                    replay_topology.is_ok(),
                    "{name} correlated-output split topology must replay: {replay_topology:?}"
                );
                let replay = result.regularized_region(&policy);
                assert!(
                    replay.is_ok(),
                    "{name} must remain an authoritative Boolean input after correlated chord splits: {replay:?}"
                );
            }
            let far_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    close(
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(
                            10, 10,
                        )),
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(
                            11, 10,
                        )),
                    ),
                    close(
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(
                            11, 10,
                        )),
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(
                            10, 11,
                        )),
                    ),
                    close(
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(
                            10, 11,
                        )),
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(
                            10, 10,
                        )),
                    ),
                ],
                &policy,
            )
            .expect("far exact triangle must close");
            let far_region = region(far_loop);
            let replay_boolean = booleans.xor().boolean_regions(&far_region, &policy);
            assert!(
                replay_boolean.is_ok(),
                "a correlated Boolean output must remain usable against a disjoint exact region: {replay_boolean:?}"
            );
            let enclosing_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    close(
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(
                            -3, -3,
                        )),
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(5, -3)),
                    ),
                    close(
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(5, -3)),
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(-3, 5)),
                    ),
                    close(
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(-3, 5)),
                        RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(
                            -3, -3,
                        )),
                    ),
                ],
                &policy,
            )
            .expect("enclosing exact triangle must close");
            let enclosing_region = region(enclosing_loop);
            let contained_context =
                CurveRegionBooleanContext::try_new(booleans.xor(), &enclosing_region, &policy)
                    .expect("valid contained replay context");
            let contained_topology = contained_context.build_boolean_topology();
            assert!(
                contained_topology.is_ok(),
                "contained correlated topology must classify: {contained_topology:?}"
            );
            let contained_replay = booleans.xor().boolean_regions(&enclosing_region, &policy);
            assert!(
                contained_replay.is_ok(),
                "a correlated Boolean output must remain classifiable inside an exact region: {contained_replay:?}"
            );
        }
    }

    #[test]
    fn noninjective_endpoint_preimages_complete_public_region_intersection() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first_parameter = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
            let second_parameter = BezierParameter2::Algebraic(sqrt_third_parameter(&policy));
            let horizontal = rational_line(0, 1);
            let first_endpoint =
                exact_contact_point_evidence(&horizontal, &first_parameter, &policy)
                    .expect("exact first endpoint")
                    .expect("first endpoint evidence");
            let second_endpoint =
                exact_contact_point_evidence(&horizontal, &second_parameter, &policy)
                    .expect("exact second endpoint")
                    .expect("second endpoint evidence");
            let bottom =
                RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(0, -1));
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    first_endpoint.clone(),
                    second_endpoint.clone(),
                    &policy,
                )
                .expect("valid independent-field chord"),
            );
            let second_closure = decided(
                crate::BezierAlgebraicChord2::try_new(second_endpoint, bottom.clone(), &policy)
                    .expect("valid second closure"),
            );
            let first_closure = decided(
                crate::BezierAlgebraicChord2::try_new(bottom, first_endpoint, &policy)
                    .expect("valid first closure"),
            );
            let chord_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::AlgebraicChord(chord),
                    BezierSplitFragment2::AlgebraicChord(second_closure),
                    BezierSplitFragment2::AlgebraicChord(first_closure),
                ],
                &policy,
            )
            .expect("independent-field chord triangle must close");
            let chord_region = CurveRegion2::try_new_with_loop_topology(
                vec![chord_loop],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid independent-field chord triangle");

            let materialized_line =
                |start: Point2, end: Point2| BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                        LineSeg2::try_new(start, end).expect("valid source closure edge"),
                    )),
                };
            let source_loop = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                            Point2::from_values(0, 0),
                            Point2::from_values(2, 0),
                            Point2::from_values(0, 0),
                        )),
                    },
                    materialized_line(Point2::from_values(0, 0), Point2::from_values(-1, 0)),
                    materialized_line(Point2::from_values(-1, 0), Point2::from_values(-1, 1)),
                    materialized_line(Point2::from_values(-1, 1), Point2::from_values(0, 1)),
                    materialized_line(Point2::from_values(0, 1), Point2::from_values(0, 0)),
                ],
                &policy,
            )
            .expect("retraced source loop must close");
            let source_region = CurveRegion2::try_new_with_loop_topology(
                vec![source_loop],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid retraced source region");

            let intersections = chord_region
                .intersect_region(&source_region, &policy)
                .expect("noninjective endpoint contacts must complete");
            assert_eq!(
                intersections.certainty,
                crate::CurveCertainty::Certified,
                "the correlated exact proof must precede the approximate terminal"
            );
            let intersections = intersections.into_value();
            assert!(intersections.is_complete(), "{intersections:?}");
            assert_eq!(intersections.overlaps().len(), 2, "{intersections:?}");
            assert_eq!(intersections.contacts().len(), 4, "{intersections:?}");
        }
    }

    #[test]
    fn noninjective_collinear_chord_dispatch_retains_contacts_and_overlaps() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let endpoint_parameter = sqrt_half_parameter(&policy);
            let horizontal = rational_line(0, 1);
            let endpoint = RationalBezierIntersectionPointEvidence2::Algebraic(
                horizontal
                    .point_at_algebraic_parameter(&endpoint_parameter, &policy)
                    .expect("exact algebraic endpoint image"),
            );
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::from_values(0, 0)),
                    endpoint,
                    &policy,
                )
                .expect("valid mixed-field chord"),
            );
            let fraction = |numerator: i8, denominator: i8| {
                (Real::from(numerator) / Real::from(denominator)).expect("nonzero denominator")
            };
            let cubic_point = |numerator, denominator| {
                Point2::new(fraction(numerator, denominator), Real::zero())
            };
            // q(t) has a double zero at 1/4, a second zero at 5/8,
            // and then rises through the complete chord image.
            let source = BezierSubcurve2::Cubic(CubicBezier2::new(
                cubic_point(-5, 27),
                cubic_point(11, 27),
                cubic_point(-7, 9),
                Point2::from_values(1, 0),
            ));
            let chord_geometry = RegionCarrierGeometry::AlgebraicChord(chord.clone());
            let source_geometry = RegionCarrierGeometry::Bezier(source);
            let empty_first = CurveRegion2::empty();
            let empty_second = CurveRegion2::empty();
            let context = CurveRegionBooleanContext {
                data: CurveRegionBooleanContextData {
                    first: &empty_first,
                    second: &empty_second,
                    policy,
                    carriers: vec![
                        RegionCarrier {
                            operand: CurveRegionBooleanOperand2::First,
                            loop_index: 0,
                            fragment_index: 0,
                            family: chord_geometry.family(),
                            geometry: chord_geometry,
                            start: CurveRegionParameter2::from_algebraic_chord(
                                chord.start_parameter(),
                            ),
                            end: CurveRegionParameter2::from_algebraic_chord(chord.end_parameter()),
                            reversed: false,
                            filled_side_is_left: true,
                            selected_fiber_endpoint_points: None,
                            image_is_injective: OnceLock::new(),
                            bounds: OnceLock::new(),
                        },
                        RegionCarrier {
                            operand: CurveRegionBooleanOperand2::Second,
                            loop_index: 0,
                            fragment_index: 0,
                            family: source_geometry.family(),
                            geometry: source_geometry,
                            start: carrier_parameter(BezierParameter2::Exact(Real::zero())),
                            end: carrier_parameter(BezierParameter2::Exact(Real::one())),
                            reversed: false,
                            filled_side_is_left: true,
                            selected_fiber_endpoint_points: None,
                            image_is_injective: OnceLock::new(),
                            bounds: OnceLock::new(),
                        },
                    ],
                    first_carrier_count: 1,
                    authored_carrier_pair_count: 1,
                    pairs: vec![RegionCarrierPair {
                        first_carrier_index: 0,
                        second_carrier_index: 1,
                        context: RegionCarrierPairContext::AlgebraicChordPair,
                    }],
                    bezier_self_intersections: Vec::new(),
                    parallel_self_intersections: Vec::new(),
                    strict_line_image_only: OnceLock::new(),
                },
            };
            let pair_result = context
                .pair_result(&context.data.pairs[0])
                .expect("mixed collinear evidence must dispatch exactly");
            assert!(pair_result.blockers.is_empty(), "{pair_result:?}");
            assert_eq!(pair_result.contacts.len(), 1, "{pair_result:?}");
            assert_eq!(pair_result.overlaps.len(), 1, "{pair_result:?}");
            assert!(!pair_result.contacts[0].is_certified_transverse());
            assert_eq!(
                pair_result.overlaps[0].orientation,
                RationalBezierOverlapOrientation2::Same
            );

            let evidence = context
                .build_intersection_evidence()
                .expect("mixed evidence must enter CurveRegion intersection output");
            assert!(evidence.is_complete(), "{evidence:?}");
            assert_eq!(evidence.contacts().len(), 1, "{evidence:?}");
            assert_eq!(evidence.overlaps().len(), 1, "{evidence:?}");
        }
    }

    #[test]
    fn adjacent_general_chord_fallback_excludes_the_authored_endpoint() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
            let source_parameter = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
            let independent_parameter = BezierParameter2::Algebraic(sqrt_third_parameter(&policy));
            let source_curve = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                Point2::new(Real::zero(), -half.clone()),
                Point2::new(half.clone(), -half.clone()),
                Point2::new(Real::one(), half.clone()),
            ));
            let source_rational =
                RationalBezier2::try_from_subcurve(&source_curve).expect("valid parabola");
            let shared_endpoint =
                exact_contact_point_evidence(&source_rational, &source_parameter, &policy)
                    .expect("exact shared endpoint")
                    .expect("shared endpoint evidence");
            let y_axis = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(0, 1))
                    .expect("valid y axis"),
            ));
            let y_rational =
                RationalBezier2::try_from_subcurve(&y_axis).expect("valid rational y axis");
            let independent_endpoint =
                exact_contact_point_evidence(&y_rational, &independent_parameter, &policy)
                    .expect("exact independent endpoint")
                    .expect("independent endpoint evidence");
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    shared_endpoint,
                    independent_endpoint.clone(),
                    &policy,
                )
                .expect("valid independent-field chord"),
            );
            let closure = decided(
                crate::BezierAlgebraicChord2::try_new(
                    independent_endpoint,
                    RationalBezierIntersectionPointEvidence2::Exact(Point2::new(
                        Real::zero(),
                        -half,
                    )),
                    &policy,
                )
                .expect("valid algebraic closure"),
            );
            let source_fragment = decided(
                source_curve
                    .split_at_parameters(std::slice::from_ref(&source_parameter), &policy)
                    .expect("exact source split"),
            )
            .fragments()[0]
                .clone();
            let boundary = CurveRegionBoundaryLoop2::new(
                vec![
                    source_fragment,
                    BezierSplitFragment2::AlgebraicChord(chord),
                    BezierSplitFragment2::AlgebraicChord(closure),
                ],
                &policy,
            )
            .expect("adjacent independent-field loop must close");
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid adjacent independent-field region");
            let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
                .expect("valid adjacent Boolean context");
            let pair = context
                .data
                .pairs
                .iter()
                .find(|pair| {
                    [pair.first_carrier_index, pair.second_carrier_index]
                        .into_iter()
                        .map(|index| &context.data.carriers[index])
                        .any(|carrier| carrier.fragment_index == 0)
                        && [pair.first_carrier_index, pair.second_carrier_index]
                            .into_iter()
                            .map(|index| &context.data.carriers[index])
                            .any(|carrier| carrier.fragment_index == 1)
                })
                .expect("adjacent source/chord endpoint must schedule the pair");
            assert!(context.authored_carriers_are_adjacent(pair));
            let result = context
                .pair_result(pair)
                .expect("adjacent independent-field fallback must complete");
            assert!(result.blockers.is_empty(), "adjacent result: {result:?}");
            assert!(
                result.contacts.is_empty(),
                "authored adjacency owns the shared endpoint: {result:?}"
            );
        }
    }

    fn shifted_sqrt_half_parameter(shift: Real, policy: &CurveContext) -> BezierParameter2 {
        let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
        let polynomial = decided(
            crate::BezierParameterPolynomial::try_new_power_basis(
                vec![
                    &shift * &shift - half,
                    Real::zero() - &shift * Real::from(2_i8),
                    Real::one(),
                ],
                policy,
            )
            .expect("valid shifted quadratic"),
        );
        let interval = decided(
            crate::BezierParameterInterval::try_new(
                (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
                Real::one(),
                policy,
            )
            .expect("valid positive-root interval"),
        );
        BezierParameter2::Algebraic(decided(
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)
                .expect("isolated shifted positive root"),
        ))
    }

    fn shifted_nested_radical_parameter(shift: Real, policy: &CurveContext) -> BezierParameter2 {
        let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
        let alpha = half.sqrt().expect("positive square root");
        let polynomial = decided(
            crate::BezierParameterPolynomial::try_new_power_basis(
                vec![
                    &shift * &shift - &shift - alpha,
                    Real::one() - &shift * Real::from(2_i8),
                    Real::one(),
                ],
                policy,
            )
            .expect("valid translated nested-radical quadratic"),
        );
        let interval = decided(
            crate::BezierParameterInterval::try_new(Real::zero(), Real::one(), policy)
                .expect("valid unit parameter interval"),
        );
        BezierParameter2::Algebraic(decided(
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)
                .expect("isolated translated nested-radical root"),
        ))
    }

    fn dyadic_epsilon(exponent: usize) -> Real {
        Real::new(
            crate::Rational::from_bigint_fraction(
                BigInt::from(1_u8),
                BigUint::from(1_u8) << exponent,
            )
            .expect("positive dyadic epsilon"),
        )
    }

    fn injective_test_carrier() -> RegionCarrier {
        RegionCarrier {
            operand: CurveRegionBooleanOperand2::First,
            loop_index: 0,
            fragment_index: 0,
            family: CurveFamily2::RationalBezier,
            geometry: RegionCarrierGeometry::Bezier(BezierSubcurve2::Rational(rational_line(0, 1))),
            start: carrier_parameter(BezierParameter2::Exact(Real::zero())),
            end: carrier_parameter(BezierParameter2::Exact(Real::one())),
            reversed: false,
            filled_side_is_left: true,
            selected_fiber_endpoint_points: None,
            image_is_injective: OnceLock::new(),
            bounds: OnceLock::new(),
        }
    }

    fn noninjective_test_carrier() -> RegionCarrier {
        RegionCarrier {
            operand: CurveRegionBooleanOperand2::First,
            loop_index: 0,
            fragment_index: 0,
            family: CurveFamily2::QuadraticBezier,
            geometry: RegionCarrierGeometry::Bezier(BezierSubcurve2::Quadratic(
                QuadraticBezier2::new(
                    Point2::from_values(0, 0),
                    Point2::from_values(1, 0),
                    Point2::from_values(0, 0),
                ),
            )),
            start: carrier_parameter(BezierParameter2::Exact(Real::zero())),
            end: carrier_parameter(BezierParameter2::Exact(Real::one())),
            reversed: false,
            filled_side_is_left: true,
            selected_fiber_endpoint_points: None,
            image_is_injective: OnceLock::new(),
            bounds: OnceLock::new(),
        }
    }

    fn cusp_test_semicircle(policy: &CurveContext) -> BezierAlgebraicCuspSemicircle2 {
        let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
        let parallel = CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(0, 4),
            Point2::from_values(4, -4),
            Point2::from_values(4, 0),
        )
        .parallel_left(half.clone())
        .expect("valid analytic parallel");
        let analysis = decided(
            parallel
                .singularity_analysis(policy)
                .expect("certified singularity analysis"),
        );
        let BezierParameter2::Algebraic(parameter) = &analysis.parallel_cusps()[0] else {
            panic!("the selected general cusp must be algebraic");
        };
        decided(
            parallel
                .algebraic_cusp_semicircle(parameter, half, false, policy)
                .expect("certified cusp semicircle"),
        )
        .expect("the nonzero cusp radius must produce a semicircle")
    }

    fn cusp_test_carrier(
        semicircle: BezierAlgebraicCuspSemicircle2,
        start: Real,
        end: Real,
        operand: CurveRegionBooleanOperand2,
        policy: &CurveContext,
    ) -> RegionCarrier {
        let fragment = decided(
            BezierAlgebraicCuspSemicircleFragment2::try_new(
                semicircle,
                BezierAlgebraicCuspSemicircleParameter2::Exact(start),
                BezierAlgebraicCuspSemicircleParameter2::Exact(end),
                false,
                policy,
            )
            .expect("valid cusp fragment"),
        );
        let geometry = RegionCarrierGeometry::AlgebraicCuspSemicircle(fragment.clone());
        RegionCarrier {
            operand,
            loop_index: 0,
            fragment_index: 0,
            family: geometry.family(),
            geometry,
            start: CurveRegionParameter2::from_algebraic_cusp(fragment.start_parameter().clone()),
            end: CurveRegionParameter2::from_algebraic_cusp(fragment.end_parameter().clone()),
            reversed: false,
            filled_side_is_left: true,
            selected_fiber_endpoint_points: None,
            image_is_injective: OnceLock::new(),
            bounds: OnceLock::new(),
        }
    }

    #[test]
    fn unary_cusp_regularization_samples_sides_in_the_selected_field() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first = cusp_test_semicircle(&policy);
            let second = first.complementary_half();
            let forward = vec![
                BezierSplitFragment2::AlgebraicCuspSemicircle(
                    BezierAlgebraicCuspSemicircleFragment2::full(first, &policy),
                ),
                BezierSplitFragment2::AlgebraicCuspSemicircle(
                    BezierAlgebraicCuspSemicircleFragment2::full(second, &policy),
                ),
            ];
            let reversed = forward
                .iter()
                .rev()
                .map(|fragment| fragment.reversed().unwrap())
                .collect::<Vec<_>>();
            for (fragments, interior_side, expected_action) in [
                (
                    forward,
                    crate::CurveBoundaryInteriorSide2::Left,
                    RegionFragmentAction::Keep,
                ),
                (
                    reversed,
                    crate::CurveBoundaryInteriorSide2::Right,
                    RegionFragmentAction::KeepReversed,
                ),
            ] {
                let boundary = CurveRegionBoundaryLoop2::new(fragments, &policy)
                    .expect("complementary selected-field cusp halves must close");
                let region = CurveRegion2::try_new_with_loop_topology(
                    vec![boundary],
                    vec![CurveRegionLoopRole::Material],
                    vec![FillRule::NonZero],
                    vec![interior_side],
                )
                .unwrap();
                let context = CurveRegionBooleanContext::try_new_unary(&region, &policy).unwrap();
                assert_eq!(context.data.carriers.len(), 2);
                for carrier_index in 0..2 {
                    let RegionCarrierGeometry::AlgebraicCuspSemicircle(fragment) =
                        &context.data.carriers[carrier_index].geometry
                    else {
                        panic!("the selected-field disk must retain both cusp carriers");
                    };
                    assert_eq!(
                        context
                            .regularized_algebraic_cusp_fragment_decision(carrier_index, fragment)
                            .expect("selected-field side rays must decide")
                            .action,
                        expected_action,
                    );
                }
            }
        }
    }

    #[test]
    fn boolean_topology_seeds_a_general_cusp_run_from_an_adjacent_carrier() {
        let local_start = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let semicircle = cusp_test_semicircle(&policy);
            let source_parameter = BezierParameter2::Algebraic(semicircle.cusp_parameter().clone());
            let regular_span = |parallel: &BezierParallel2| {
                let range = match BezierParameterRange2::try_new(
                    BezierParameter2::Exact(local_start.clone()),
                    source_parameter.clone(),
                    &policy,
                )
                .unwrap()
                {
                    Classification::Decided(range) => range,
                    Classification::Uncertain(reason) => panic!("local cusp range: {reason:?}"),
                };
                match BezierParallelFragment2::try_new(parallel.clone(), range, &policy).unwrap() {
                    Classification::Decided(fragment) => fragment,
                    Classification::Uncertain(reason) => {
                        panic!("regular local cusp span: {reason:?}")
                    }
                }
            };
            let start_parallel = semicircle
                .start_parallel()
                .expect("an analytic cusp retains its starting parallel");
            let end_parallel = semicircle
                .end_parallel()
                .expect("an analytic cusp retains its ending parallel");
            let start_fragment = regular_span(&start_parallel);
            let end_fragment = regular_span(&end_parallel);
            let local_point = |parallel: &BezierParallel2| {
                let Classification::Decided(point) =
                    parallel.point_at(&local_start, &policy).unwrap()
                else {
                    panic!("a rational local parameter must have an exact parallel image");
                };
                point
            };
            let closing_line = BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(local_point(&end_parallel), local_point(&start_parallel))
                        .unwrap(),
                )),
            };
            let boundary = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::AlgebraicCuspSemicircle(
                        BezierAlgebraicCuspSemicircleFragment2::full(semicircle, &policy),
                    ),
                    BezierSplitFragment2::AnalyticParallel(end_fragment)
                        .reversed()
                        .unwrap(),
                    closing_line,
                    BezierSplitFragment2::AnalyticParallel(start_fragment),
                ],
                &policy,
            )
            .expect("the algebraic cusp and adjacent carriers form one exact loop");
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::CurveBoundaryInteriorSide2::Left],
            )
            .unwrap();
            let distant_algebraic_point =
                RationalBezierAlgebraicPointImage2::from_parametric_source(
                    rational_line(100, 101),
                    sqrt_half_parameter(&policy),
                    &policy,
                );
            assert_eq!(
                region
                    .classify_algebraic_point_raw(&distant_algebraic_point, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Outside),
            );
            let empty = CurveRegion2::default();
            let topology = CurveRegionBooleanContext::try_new(&region, &empty, &policy)
                .unwrap()
                .build_boolean_topology()
                .expect("an adjacent exact carrier must seed the non-rational cusp run");
            assert_eq!(topology.point_classification_count, 1);
            assert!(
                topology
                    .split_fragments
                    .iter()
                    .flatten()
                    .all(|fragment| { fragment.location == Some(RegionPointLocation::Outside) })
            );
        }
    }

    #[test]
    fn affine_region_classifies_a_general_algebraic_cusp_point_in_its_source_field() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let fragment = BezierAlgebraicCuspSemicircleFragment2::full(
                cusp_test_semicircle(&policy),
                &policy,
            );
            let Classification::Decided(point) = fragment.representative_point().unwrap() else {
                panic!("the selected cusp representative must retain an exact point image");
            };
            assert!(point.exact_rational_point(&policy).is_none());
            assert_eq!(
                square_region(-10, -10, 10, 10)
                    .classify_algebraic_point_raw(&point, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Inside),
            );
            assert_eq!(
                square_region(20, 20, 30, 30)
                    .classify_algebraic_point_raw(&point, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Outside),
            );

            let curved_cap = |control_y: i8| {
                let left = Point2::from_values(-10, -10);
                let right = Point2::from_values(10, -10);
                CurveRegion2::try_from_boundary_paths_with_loop_topology(
                    &[CurvePath2::try_new(vec![
                        Curve2::from(LineSeg2::try_new(left.clone(), right.clone()).unwrap()),
                        Curve2::from(QuadraticBezier2::new(
                            right,
                            Point2::from_values(0, control_y),
                            left,
                        )),
                    ])
                    .unwrap()],
                    &[CurveRegionLoopRole::Material],
                    &[FillRule::NonZero],
                    &[crate::CurveBoundaryInteriorSide2::Left],
                    &policy,
                )
                .unwrap()
                .into_value()
            };
            assert_eq!(
                curved_cap(30)
                    .classify_algebraic_point_off_boundary_raw(&point, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Inside),
            );
            assert_eq!(
                curved_cap(0)
                    .classify_algebraic_point_off_boundary_raw(&point, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Outside),
            );

            let parameter = sqrt_half_parameter(&policy);
            let boundary = RationalBezierAlgebraicPointImage2::from_parametric_source(
                rational_line(0, 1),
                parameter.clone(),
                &policy,
            );
            assert_eq!(
                square_region(0, 0, 2, 2)
                    .classify_algebraic_point_raw(&boundary, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Boundary),
            );
            let parabola = RationalBezier2::try_new(
                vec![
                    Point2::from_values(0, 0),
                    Point2::new((Real::one() / Real::from(2_i8)).unwrap(), Real::zero()),
                    Point2::from_values(1, 1),
                ],
                vec![Real::one(); 3],
            )
            .unwrap();
            let parabola_boundary = RationalBezierAlgebraicPointImage2::from_parametric_source(
                parabola.clone(),
                parameter.clone(),
                &policy,
            );
            let parabola_region = CurveRegion2::try_from_boundary_paths_with_loop_topology(
                &[CurvePath2::try_new(vec![
                    Curve2::from(parabola),
                    Curve2::from(
                        LineSeg2::try_new(Point2::from_values(1, 1), Point2::from_values(0, 1))
                            .unwrap(),
                    ),
                    Curve2::from(
                        LineSeg2::try_new(Point2::from_values(0, 1), Point2::from_values(0, 0))
                            .unwrap(),
                    ),
                ])
                .unwrap()],
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
                &[crate::CurveBoundaryInteriorSide2::Left],
                &policy,
            )
            .unwrap()
            .into_value();
            let parabola_classification = parabola_region
                .classify_algebraic_point(&parabola_boundary, &policy)
                .unwrap();
            assert_eq!(
                parabola_classification.certainty,
                crate::CurveCertainty::Certified,
            );
            assert_eq!(
                parabola_classification.value,
                Classification::Decided(RegionPointLocation::Boundary),
            );
            let dyadic_ray_point = RationalBezierAlgebraicPointImage2::from_parametric_source(
                rational_line(0, 1),
                parameter.clone(),
                &policy,
            );
            let lower_left = Point2::from_values(-2, -1);
            let lower_right = Point2::from_values(2, -1);
            let upper_right = Point2::from_values(2, 1);
            let upper_left = Point2::from_values(-2, 1);
            let dyadic_crossing_region = CurveRegion2::try_from_boundary_paths_with_loop_topology(
                &[CurvePath2::try_new(vec![
                    Curve2::from(
                        LineSeg2::try_new(lower_left.clone(), lower_right.clone()).unwrap(),
                    ),
                    Curve2::from(QuadraticBezier2::new(
                        lower_right,
                        Point2::from_values(0, 0),
                        upper_right.clone(),
                    )),
                    Curve2::from(LineSeg2::try_new(upper_right, upper_left.clone()).unwrap()),
                    Curve2::from(LineSeg2::try_new(upper_left, lower_left).unwrap()),
                ])
                .unwrap()],
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
                &[crate::CurveBoundaryInteriorSide2::Left],
                &policy,
            )
            .unwrap()
            .into_value();
            assert_eq!(
                dyadic_crossing_region
                    .classify_algebraic_point_off_boundary_raw(&dyadic_ray_point, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Inside),
            );
            let ninth = (Real::one() / Real::from(9_i8)).unwrap();
            let tangent_start = Point2::new(Real::from(2_i8), ninth.clone());
            let tangent_end = Point2::new(Real::from(2_i8), &ninth * Real::from(4_i8));
            let tangent_upper_right = Point2::new(Real::from(3_i8), &ninth * Real::from(4_i8));
            let tangent_lower_right = Point2::new(Real::from(3_i8), ninth.clone());
            let tangent_region = CurveRegion2::try_from_boundary_paths_with_loop_topology(
                &[CurvePath2::try_new(vec![
                    Curve2::from(QuadraticBezier2::new(
                        tangent_start.clone(),
                        Point2::new(Real::zero(), -(&ninth * Real::from(2_i8))),
                        tangent_end.clone(),
                    )),
                    Curve2::from(
                        LineSeg2::try_new(tangent_end, tangent_upper_right.clone()).unwrap(),
                    ),
                    Curve2::from(
                        LineSeg2::try_new(tangent_upper_right, tangent_lower_right.clone())
                            .unwrap(),
                    ),
                    Curve2::from(LineSeg2::try_new(tangent_lower_right, tangent_start).unwrap()),
                ])
                .unwrap()],
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
                &[crate::CurveBoundaryInteriorSide2::Right],
                &policy,
            )
            .unwrap()
            .into_value();
            assert_eq!(
                tangent_region
                    .classify_algebraic_point_off_boundary_raw(&dyadic_ray_point, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Outside),
            );
            let line_extension = RationalBezierAlgebraicPointImage2::from_parametric_source(
                rational_line(2, 4),
                parameter.clone(),
                &policy,
            );
            assert_eq!(
                square_region(0, 0, 2, 2)
                    .classify_algebraic_point_raw(&line_extension, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Outside),
            );

            let negative_denominator = RationalBezierAlgebraicPointImage2::from_retained_expression(
                parameter.clone(),
                crate::bezier_algebraic_image::parameter_representation(&parameter, &policy),
                vec![Real::zero(), Real::from(-1_i8)],
                vec![Real::zero(), Real::from(-1_i8)],
                vec![Real::from(-1_i8)],
                "test a correlated point with negative projective weight",
            );
            assert_eq!(
                square_region(0, 0, 2, 2)
                    .classify_algebraic_point_raw(&negative_denominator, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Inside),
            );

            let outer = square_region(-2, -2, 2, 2);
            let hole = square_region(0, 0, 1, 1);
            let without_arrangement_provenance = |region: &CurveRegion2| {
                CurveRegionBoundaryLoop2::new(
                    region.boundary_loops()[0].fragments().to_vec(),
                    &policy,
                )
                .unwrap()
            };
            let holed = CurveRegion2::try_new_with_loop_topology(
                vec![
                    without_arrangement_provenance(&outer),
                    without_arrangement_provenance(&hole),
                ],
                vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole],
                vec![FillRule::NonZero; 2],
                vec![
                    crate::CurveBoundaryInteriorSide2::Left,
                    crate::CurveBoundaryInteriorSide2::Right,
                ],
            )
            .unwrap();
            assert_eq!(
                holed
                    .classify_algebraic_point_raw(&negative_denominator, &policy)
                    .unwrap(),
                Classification::Decided(RegionPointLocation::Outside),
            );
        }
    }

    #[test]
    fn cusp_overlap_clipping_maps_partial_carriers_in_both_orientations() {
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let two_fifths = (Real::from(2_i8) / Real::from(5_i8)).unwrap();
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let three_fifths = (Real::from(3_i8) / Real::from(5_i8)).unwrap();
        let three_quarters = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first = cusp_test_semicircle(&policy);
            for (second, second_start, second_end, expected_first, expected_second) in [
                (
                    first.clone(),
                    half.clone(),
                    Real::one(),
                    (half.clone(), three_quarters.clone()),
                    (half.clone(), three_quarters.clone()),
                ),
                (
                    first.reversed(),
                    Real::zero(),
                    two_fifths.clone(),
                    (three_fifths.clone(), three_quarters.clone()),
                    (two_fifths.clone(), quarter.clone()),
                ),
            ] {
                let Classification::Decided(
                    BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(overlap),
                ) = first.pair_intersections(&second, &policy).unwrap()
                else {
                    panic!("coincident selected semicircles must overlap");
                };
                let first_carrier = cusp_test_carrier(
                    first.clone(),
                    quarter.clone(),
                    three_quarters.clone(),
                    CurveRegionBooleanOperand2::First,
                    &policy,
                );
                let second_carrier = cusp_test_carrier(
                    second,
                    second_start,
                    second_end,
                    CurveRegionBooleanOperand2::Second,
                    &policy,
                );
                let Classification::Decided(Some(clipped)) = overlap
                    .clipped_curve_region_ranges(
                        &CurveRegionParameterRange2::new_validated(
                            first_carrier.start.clone(),
                            first_carrier.end.clone(),
                        ),
                        &CurveRegionParameterRange2::new_validated(
                            second_carrier.start.clone(),
                            second_carrier.end.clone(),
                        ),
                        &policy,
                    )
                    .unwrap()
                else {
                    panic!("the carrier fragments retain a positive shared span");
                };
                assert_eq!(
                    clipped.0.exact_endpoints(),
                    Some((&expected_first.0, &expected_first.1)),
                );
                assert_eq!(
                    clipped.1.exact_endpoints(),
                    Some((&expected_second.0, &expected_second.1)),
                );
            }
        }
    }

    #[test]
    fn injective_carrier_topology_vertex_canonicalizes_unorderable_parameter_aliases() {
        let policy = CurveContext::STRICT;
        let first = shifted_nested_radical_parameter(Real::zero(), &policy);
        let second = shifted_nested_radical_parameter(dyadic_epsilon(600), &policy);
        assert_eq!(
            first.cmp_by_refinement(&second, &policy).unwrap(),
            Classification::Uncertain(UncertaintyReason::Ordering),
        );

        let carrier = injective_test_carrier();
        let mut events = Vec::new();
        push_carrier_event(
            &mut events,
            carrier_parameter(first.clone()),
            Some(7),
            &carrier,
            &policy,
        )
        .unwrap();
        push_carrier_event(
            &mut events,
            carrier_parameter(second.clone()),
            Some(7),
            &carrier,
            &policy,
        )
        .unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].parameter.as_bezier_parameter(), Some(&first));

        assert!(matches!(
            push_carrier_event(
                &mut events,
                carrier_parameter(second),
                Some(8),
                &carrier,
                &policy
            ),
            Err(ExactCurveError::Blocked(_)),
        ));

        let approximate = CurveContext::APPROXIMATE_512;
        let first = shifted_nested_radical_parameter(Real::zero(), &approximate);
        let second = shifted_nested_radical_parameter(dyadic_epsilon(600), &approximate);
        let carrier = injective_test_carrier();
        let mut events = Vec::new();
        push_carrier_event(
            &mut events,
            carrier_parameter(first),
            Some(7),
            &carrier,
            &approximate,
        )
        .unwrap();
        push_carrier_event(
            &mut events,
            carrier_parameter(second),
            Some(7),
            &carrier,
            &approximate,
        )
        .unwrap();
        assert_eq!(events.len(), 1);
    }

    #[test]
    fn parameter_order_uses_exact_translation_before_approximate_512_terminal() {
        let strict = CurveContext::STRICT;
        let first = shifted_sqrt_half_parameter(Real::zero(), &strict);
        let second = shifted_sqrt_half_parameter(dyadic_epsilon(600), &strict);
        assert_eq!(
            first.cmp_by_refinement(&second, &strict).unwrap(),
            Classification::Decided(Ordering::Less),
        );

        let approximate = CurveContext::APPROXIMATE_512;
        let first = shifted_sqrt_half_parameter(Real::zero(), &approximate);
        let second = shifted_sqrt_half_parameter(dyadic_epsilon(600), &approximate);
        let outcome = crate::policy::resolve_certified_operation(&approximate, |attempt| {
            first.cmp_by_refinement(&second, attempt)
        })
        .unwrap();
        assert_eq!(outcome.value, Classification::Decided(Ordering::Less));
        assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
    }

    #[test]
    fn unsupported_parameter_order_obeys_strict_and_approximate_512_policies() {
        let strict = CurveContext::STRICT;
        let first = shifted_nested_radical_parameter(Real::zero(), &strict);
        let second = shifted_nested_radical_parameter(dyadic_epsilon(600), &strict);
        assert_eq!(
            first.cmp_by_refinement(&second, &strict).unwrap(),
            Classification::Uncertain(UncertaintyReason::Ordering),
        );

        let approximate = CurveContext::APPROXIMATE_512;
        let first = shifted_nested_radical_parameter(Real::zero(), &approximate);
        let second = shifted_nested_radical_parameter(dyadic_epsilon(600), &approximate);
        let outcome = crate::policy::resolve_certified_operation(&approximate, |attempt| {
            first.cmp_by_refinement(&second, attempt)
        })
        .unwrap();
        assert_eq!(outcome.value, Classification::Decided(Ordering::Equal));
        assert_eq!(
            outcome.certainty,
            crate::CurveCertainty::Approximate512Consumed
        );
    }

    #[test]
    fn noninjective_carrier_retains_distinct_branches_at_one_topology_vertex() {
        let policy = CurveContext::STRICT;
        let carrier = noninjective_test_carrier();
        let mut events = Vec::new();
        push_carrier_event(
            &mut events,
            carrier_parameter(BezierParameter2::Exact(
                (Real::one() / Real::from(4_i8)).expect("nonzero denominator"),
            )),
            Some(7),
            &carrier,
            &policy,
        )
        .unwrap();
        push_carrier_event(
            &mut events,
            carrier_parameter(BezierParameter2::Exact(
                (Real::from(3_i8) / Real::from(4_i8)).expect("nonzero denominator"),
            )),
            Some(7),
            &carrier,
            &policy,
        )
        .unwrap();
        assert_eq!(events.len(), 2);

        let mut all_events = vec![events];
        canonicalize_injective_topology_events(
            &mut all_events,
            std::slice::from_ref(&carrier),
            &policy,
        );
        assert_eq!(all_events[0].len(), 2);
    }

    #[test]
    fn polynomial_control_polygon_certifies_only_monotone_injective_axis() {
        let monotone = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 1),
            Point2::from_values(2, 0),
        ));
        let retraced = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::from_values(0, 0),
        ));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert!(monotone.has_certified_injective_axis(&policy));
            assert!(!retraced.has_certified_injective_axis(&policy));
        }
    }

    #[test]
    fn transitive_topology_merge_canonicalizes_deferred_injective_aliases() {
        let policy = CurveContext::STRICT;
        let first = shifted_nested_radical_parameter(Real::zero(), &policy);
        let second = shifted_nested_radical_parameter(dyadic_epsilon(600), &policy);
        let carriers = [
            injective_test_carrier(),
            injective_test_carrier(),
            injective_test_carrier(),
        ];
        let quarter =
            BezierParameter2::Exact((Real::one() / Real::from(4_i8)).expect("nonzero denominator"));
        let mut events = vec![Vec::new(), Vec::new(), Vec::new()];
        push_contact_carrier_event(
            &mut events[0],
            carrier_parameter(first.clone()),
            Some(1),
            &carriers[0],
            &policy,
        )
        .unwrap();
        push_contact_carrier_event(
            &mut events[1],
            carrier_parameter(quarter.clone()),
            Some(1),
            &carriers[1],
            &policy,
        )
        .unwrap();
        push_contact_carrier_event(
            &mut events[0],
            carrier_parameter(second.clone()),
            Some(2),
            &carriers[0],
            &policy,
        )
        .unwrap();
        push_contact_carrier_event(
            &mut events[2],
            carrier_parameter(quarter.clone()),
            Some(2),
            &carriers[2],
            &policy,
        )
        .unwrap();
        assert_eq!(events[0].len(), 2);

        let mut contacts = vec![
            ContactVertex {
                point: None,
                topology_vertex: 1,
                carrier_indices: [0, 1],
                parameters: [carrier_parameter(first), carrier_parameter(quarter.clone())],
            },
            ContactVertex {
                point: None,
                topology_vertex: 2,
                carrier_indices: [0, 2],
                parameters: [carrier_parameter(second), carrier_parameter(quarter)],
            },
        ];
        replace_topology_vertex(&mut events, &mut contacts, 2, 1);
        canonicalize_injective_topology_events(&mut events, &carriers, &policy);
        validate_carrier_event_separation(&events, &carriers, &policy).unwrap();
        assert_eq!(events[0].len(), 1);
        assert!(contacts.iter().all(|contact| contact.topology_vertex == 1));
    }

    fn assert_monotone_parallel_pair_proofs_match_complete_solver(
        context: &CurveRegionBooleanContext,
        filled_side_is_left: bool,
    ) {
        assert_eq!(context.data.pairs.len(), 6);
        assert!(context.data.pairs.iter().all(|pair| {
            context.parallel_pair_is_coordinate_disjoint(pair)
                || context.adjacent_parallel_pair_is_endpoint_only(pair)
        }));
        assert!(
            context
                .data
                .pairs
                .iter()
                .all(|pair| { !matches!(pair.context, RegionCarrierPairContext::ParallelSelf) })
        );
        let topology = context
            .build_split_topology()
            .expect("the monotone loop topology must complete");
        assert_eq!(
            context.certified_simple_single_loop_filled_side(&topology),
            Some(filled_side_is_left)
        );

        // Differentially replay the complete pair solver behind every
        // structural omission. Coordinate separation must remove all retained
        // contacts; an adjacent-range proof may leave only the loop vertex
        // that construction already seeded. This keeps the fast proof a
        // specialization of the same authority rather than an alternate
        // intersection definition.
        for replay_policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for pair in &context.data.pairs {
                let first = &context.data.carriers[pair.first_carrier_index];
                let second = &context.data.carriers[pair.second_carrier_index];
                let coordinate_disjoint = context.parallel_pair_is_coordinate_disjoint(pair);
                let adjacent_endpoint = context.adjacent_parallel_pair_is_endpoint_only(pair);
                assert!(coordinate_disjoint || adjacent_endpoint);
                let intersections = first
                    .geometry
                    .parallel()
                    .parallel_intersections(second.geometry.parallel(), &replay_policy)
                    .expect("the complete analytic-parallel replay is valid");
                let Classification::Decided(intersections) = intersections else {
                    panic!("a structurally omitted pair must have a complete exact replay");
                };
                assert!(intersections.is_complete(), "{intersections:?}");
                assert!(intersections.parameter_components().is_empty());
                assert!(intersections.overlaps().iter().all(|overlap| {
                    !(ranges_intersect(
                        &carrier_range(overlap.first_range()),
                        first,
                        &replay_policy,
                    )
                    .expect("the first overlap range comparison is decided")
                        && ranges_intersect(
                            &carrier_range(overlap.second_range()),
                            second,
                            &replay_policy,
                        )
                        .expect("the second overlap range comparison is decided"))
                }));
                let retained_contacts = intersections
                    .contacts()
                    .iter()
                    .filter(|contact| {
                        parameter_in_carrier(
                            &carrier_parameter(contact.first_parameter().clone()),
                            first,
                            &replay_policy,
                        )
                        .expect("the first contact range comparison is decided")
                            && parameter_in_carrier(
                                &carrier_parameter(contact.second_parameter().clone()),
                                second,
                                &replay_policy,
                            )
                            .expect("the second contact range comparison is decided")
                    })
                    .collect::<Vec<_>>();
                if coordinate_disjoint {
                    assert!(retained_contacts.is_empty(), "{retained_contacts:?}");
                    continue;
                }

                let fragment_count = context.data.first.boundary_loops()[first.loop_index]
                    .fragments()
                    .len();
                let expected = if first.fragment_index.checked_add(1) == Some(second.fragment_index)
                {
                    (
                        carrier_traversal_end_parameter(first),
                        carrier_traversal_start_parameter(second),
                    )
                } else {
                    assert_eq!(first.fragment_index, 0);
                    assert_eq!(second.fragment_index.checked_add(1), Some(fragment_count));
                    (
                        carrier_traversal_start_parameter(first),
                        carrier_traversal_end_parameter(second),
                    )
                };
                assert!(retained_contacts.len() <= 1, "{retained_contacts:?}");
                assert!(retained_contacts.iter().all(|contact| {
                    Some(contact.first_parameter()) == expected.0.as_bezier_parameter()
                        && Some(contact.second_parameter()) == expected.1.as_bezier_parameter()
                }));
            }
        }
    }

    #[test]
    fn monotone_parallel_ranges_remove_only_proven_unary_pairs() {
        let policy = CurveContext::STRICT;
        let tenth = (Real::one() / Real::from(10_i8)).expect("nonzero denominator");
        let sources = [
            QuadraticBezier2::new(
                Point2::from_values(1, 0),
                Point2::from_values(1, 1),
                Point2::from_values(0, 1),
            ),
            QuadraticBezier2::new(
                Point2::from_values(0, 1),
                Point2::from_values(-1, 1),
                Point2::from_values(-1, 0),
            ),
            QuadraticBezier2::new(
                Point2::from_values(-1, 0),
                Point2::from_values(-1, -1),
                Point2::from_values(0, -1),
            ),
            QuadraticBezier2::new(
                Point2::from_values(0, -1),
                Point2::from_values(1, -1),
                Point2::from_values(1, 0),
            ),
        ];
        let fragments = sources
            .into_iter()
            .map(|source| {
                let parallel = source
                    .parallel_left(-tenth.clone())
                    .expect("valid exact parallel");
                BezierSplitFragment2::AnalyticParallel(
                    BezierParallelFragment2::from_certified_range(
                        parallel,
                        BezierParameterRange2::new_validated(
                            BezierParameter2::Exact(Real::zero()),
                            BezierParameter2::Exact(Real::one()),
                        ),
                        false,
                    ),
                )
            })
            .collect();
        let region = CurveRegion2::new(vec![
            CurveRegionBoundaryLoop2::new(fragments, &policy)
                .expect("connected exact parallel loop"),
        ])
        .expect("valid raw region")
        .with_certified_loop_roles(vec![CurveRegionLoopRole::Material])
        .expect("valid material role")
        .with_certified_filled_side_is_left(vec![true])
        .expect("valid filled-side evidence");
        let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
            .expect("valid unary arrangement");

        assert_monotone_parallel_pair_proofs_match_complete_solver(&context, true);

        let reversed_fragments = region.boundary_loops()[0]
            .fragments()
            .iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("exact traversal reversal"))
            .collect();
        let reversed_region = CurveRegion2::new(vec![
            CurveRegionBoundaryLoop2::new(reversed_fragments, &policy)
                .expect("connected reversed exact parallel loop"),
        ])
        .expect("valid reversed raw region")
        .with_certified_loop_roles(vec![CurveRegionLoopRole::Material])
        .expect("valid reversed material role")
        .with_certified_filled_side_is_left(vec![false])
        .expect("valid reversed filled-side evidence");
        let reversed_context = CurveRegionBooleanContext::try_new_unary(&reversed_region, &policy)
            .expect("valid reversed unary arrangement");
        assert_monotone_parallel_pair_proofs_match_complete_solver(&reversed_context, false);
    }

    fn square_region(min_x: i8, min_y: i8, max_x: i8, max_y: i8) -> CurveRegion2 {
        let points = [
            Point2::from_values(min_x, min_y),
            Point2::from_values(max_x, min_y),
            Point2::from_values(max_x, max_y),
            Point2::from_values(min_x, max_y),
        ];
        let curves = (0..points.len())
            .map(|index| {
                Curve2::from(
                    LineSeg2::try_new(
                        points[index].clone(),
                        points[(index + 1) % points.len()].clone(),
                    )
                    .unwrap(),
                )
            })
            .collect();
        CurveRegion2::try_from_boundary_paths(
            &[CurvePath2::try_new(curves).unwrap()],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    }

    #[test]
    fn native_region_fast_path_matches_forced_general_arrangement() {
        let first = square_region(0, 0, 4, 4);
        let second = square_region(2, 0, 6, 4);
        let policy = CurveContext::STRICT;
        let operations = [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ];
        let fast = operations.map(|operation| {
            first
                .boolean_region_raw(&second, operation, &policy)
                .unwrap()
        });

        let general = CurveRegionBooleanContext::try_new(&first, &second, &policy)
            .unwrap()
            .build_boolean_regions()
            .unwrap();
        assert!(general.candidate_carrier_pair_count() > 0);
        assert!(general.topology_fragment_count() > 0);
        for (operation, fast_region) in operations.into_iter().zip(&fast) {
            let general_region = general.region(operation);
            assert_eq!(
                decided(fast_region.signed_area(&policy).unwrap().into_value()),
                decided(general_region.signed_area(&policy).unwrap().into_value())
            );
            for x_numerator in -2_i8..=14 {
                for y_numerator in -2_i8..=10 {
                    let point = Point2::new(
                        (Real::from(x_numerator) / Real::from(2_i8)).unwrap(),
                        (Real::from(y_numerator) / Real::from(2_i8)).unwrap(),
                    );
                    assert_eq!(
                        fast_region
                            .classify_point(&point, &policy)
                            .unwrap()
                            .into_value(),
                        general_region
                            .classify_point(&point, &policy)
                            .unwrap()
                            .into_value(),
                        "forced-general {operation:?} differs at {point:?}"
                    );
                }
            }
        }
    }

    fn direction(
        carrier_index: usize,
        follows_carrier: bool,
    ) -> BooleanArrangementFragmentDirection {
        BooleanArrangementFragmentDirection {
            carrier_index,
            follows_carrier,
            start_contact_branch: None,
            end_contact_branch: None,
        }
    }

    fn contact_direction(carrier_index: usize, follows_carrier: bool) -> CertifiedContactDirection {
        CertifiedContactDirection {
            branch: if carrier_index == 3 {
                TransitionContactBranch::First
            } else {
                TransitionContactBranch::Second
            },
            follows_carrier,
        }
    }

    fn vector(
        direction: BooleanArrangementFragmentDirection,
        crossing_is_positive: bool,
    ) -> (i8, i8) {
        let vector = if direction.carrier_index == 3 {
            (1, 0)
        } else if crossing_is_positive {
            (0, 1)
        } else {
            (0, -1)
        };
        if direction.follows_carrier {
            vector
        } else {
            (-vector.0, -vector.1)
        }
    }

    fn numerical_turn_preference(
        base: (i8, i8),
        first: (i8, i8),
        second: (i8, i8),
    ) -> Option<bool> {
        let half = |candidate: (i8, i8)| {
            let cross = base.0 * candidate.1 - base.1 * candidate.0;
            if cross > 0 {
                0
            } else if cross < 0 {
                1
            } else if base.0 * candidate.0 + base.1 * candidate.1 > 0 {
                0
            } else {
                1
            }
        };
        let first_half = half(first);
        let second_half = half(second);
        if first_half != second_half {
            return Some(first_half < second_half);
        }
        match (first.0 * second.1 - first.1 * second.0).cmp(&0) {
            Ordering::Greater => Some(true),
            Ordering::Less => Some(false),
            Ordering::Equal => None,
        }
    }

    #[test]
    fn classified_crossing_side_recovers_oriented_tangent_cross() {
        assert_eq!(
            transverse_cross_from_locations(
                RegionPointLocation::Outside,
                RegionPointLocation::Inside,
                true,
            ),
            Some(true)
        );
        assert_eq!(
            transverse_cross_from_locations(
                RegionPointLocation::Inside,
                RegionPointLocation::Outside,
                true,
            ),
            Some(false)
        );
        assert_eq!(
            transverse_cross_from_locations(
                RegionPointLocation::Outside,
                RegionPointLocation::Inside,
                false,
            ),
            Some(false)
        );
        assert_eq!(
            transverse_cross_from_locations(
                RegionPointLocation::Inside,
                RegionPointLocation::Outside,
                false,
            ),
            Some(true)
        );
    }

    #[test]
    fn transverse_contact_certificates_seed_both_operand_faces() {
        fn split(start: usize, end: usize) -> ClassifiedSplitCarrierFragment {
            ClassifiedSplitCarrierFragment {
                split: SplitCarrierFragment {
                    fragment: BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                                .unwrap(),
                        )),
                    },
                    start_topology_vertex: Some(start),
                    end_topology_vertex: Some(end),
                },
                location: None,
            }
        }

        let policy = CurveContext::STRICT;
        let first_region = square_region(0, 0, 2, 2);
        let second_region = square_region(1, 1, 3, 3);
        let mut context =
            CurveRegionBooleanContext::try_new(&first_region, &second_region, &policy).unwrap();
        let first_carrier = 0;
        let second_carrier = context.data.first_carrier_count;
        let vertex = 17;
        for source_cross_is_positive in [false, true] {
            for first_reversed in [false, true] {
                for second_reversed in [false, true] {
                    for first_filled_left in [false, true] {
                        for second_filled_left in [false, true] {
                            context.data.carriers[first_carrier].reversed = first_reversed;
                            context.data.carriers[second_carrier].reversed = second_reversed;
                            context.data.carriers[first_carrier].filled_side_is_left =
                                first_filled_left;
                            context.data.carriers[second_carrier].filled_side_is_left =
                                second_filled_left;
                            let mut fragments = vec![Vec::new(); context.data.carriers.len()];
                            fragments[first_carrier] = vec![split(1, vertex), split(vertex, 2)];
                            fragments[second_carrier] = vec![split(3, vertex), split(vertex, 4)];
                            let contacts = HashMap::from([(
                                vertex,
                                TransitionContactCandidate {
                                    first_carrier,
                                    second_carrier,
                                    interior_on_both_carriers: true,
                                    certified_transverse: true,
                                    cross_is_positive: Some(source_cross_is_positive),
                                    tangent_dot_is_positive: None,
                                    second_side_of_first: None,
                                    self_parameters: None,
                                },
                            )]);
                            context
                                .seed_transverse_boolean_locations(&mut fragments, &contacts)
                                .unwrap();

                            let traversal_cross_is_positive =
                                source_cross_is_positive ^ first_reversed ^ second_reversed;
                            let first_before =
                                boolean_location(traversal_cross_is_positive == second_filled_left);
                            let second_before =
                                boolean_location(traversal_cross_is_positive != first_filled_left);
                            assert_eq!(fragments[first_carrier][0].location, Some(first_before));
                            assert_eq!(
                                fragments[first_carrier][1].location,
                                toggled_region_location(first_before),
                            );
                            assert_eq!(fragments[second_carrier][0].location, Some(second_before),);
                            assert_eq!(
                                fragments[second_carrier][1].location,
                                toggled_region_location(second_before),
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn contact_point_bounds_reject_disjoint_lazy_sources() {
        let policy = CurveContext::STRICT;
        let parameter = sqrt_half_parameter(&policy);
        let first_curve = rational_line(0, 1);
        let second_curve = rational_line(2, 3);
        let first = RationalBezierIntersectionPointEvidence2::Algebraic(
            RationalBezierAlgebraicPointImage2::from_parametric_source(
                first_curve.clone(),
                parameter.clone(),
                &policy,
            ),
        );
        let second = RationalBezierIntersectionPointEvidence2::Algebraic(
            RationalBezierAlgebraicPointImage2::from_parametric_source(
                second_curve.clone(),
                parameter.clone(),
                &policy,
            ),
        );

        assert!(
            parameter
                .cached_rational_bezier_point_image(&first_curve)
                .is_none()
        );
        assert!(
            parameter
                .cached_rational_bezier_point_image(&second_curve)
                .is_none()
        );
        assert_eq!(
            first.same_point(&second, &policy),
            Classification::Decided(false)
        );
        assert!(
            parameter
                .cached_rational_bezier_point_image(&first_curve)
                .is_none()
        );
        assert!(
            parameter
                .cached_rational_bezier_point_image(&second_curve)
                .is_none()
        );
    }

    #[test]
    fn identical_injective_source_parameters_compare_without_materialization() {
        let policy = CurveContext::STRICT;
        let parameter = sqrt_half_parameter(&policy);
        let curve = rational_line(0, 1);
        let first = RationalBezierIntersectionPointEvidence2::Algebraic(
            RationalBezierAlgebraicPointImage2::from_parametric_source(
                curve.clone(),
                parameter.clone(),
                &policy,
            ),
        );
        let second = RationalBezierIntersectionPointEvidence2::Algebraic(
            RationalBezierAlgebraicPointImage2::from_parametric_source(
                curve.clone(),
                parameter.clone(),
                &policy,
            ),
        );

        assert_eq!(
            first.same_point(&second, &policy),
            Classification::Decided(true)
        );
        assert!(
            parameter
                .cached_rational_bezier_point_image(&curve)
                .is_none()
        );
    }

    #[test]
    fn nontransverse_point_touch_certifies_authored_loop_successors() {
        let split = |start_x, end_x, start_vertex, end_vertex| SplitCarrierFragment {
            fragment: BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(
                        Point2::from_values(start_x, 0),
                        Point2::from_values(end_x, 0),
                    )
                    .unwrap(),
                )),
            },
            start_topology_vertex: Some(start_vertex),
            end_topology_vertex: Some(end_vertex),
        };
        let source_splits = [
            split(-1, 0, 0, 7),
            split(0, -1, 7, 0),
            split(1, 0, 1, 7),
            split(0, 1, 7, 1),
        ];
        let topology = CurveRegionBooleanTopology {
            split_fragments: source_splits
                .iter()
                .cloned()
                .map(|split| {
                    vec![ClassifiedSplitCarrierFragment {
                        split,
                        location: Some(RegionPointLocation::Outside),
                    }]
                })
                .collect(),
            overlaps: Vec::new(),
            transverse_contacts: HashMap::new(),
            point_classification_count: 0,
        };
        let mut carriers = (0..4).map(|_| injective_test_carrier()).collect::<Vec<_>>();
        for (index, carrier) in carriers.iter_mut().enumerate() {
            carrier.operand = if index < 2 {
                CurveRegionBooleanOperand2::First
            } else {
                CurveRegionBooleanOperand2::Second
            };
            carrier.fragment_index = index % 2;
        }

        for follows_carrier in [false, true] {
            let source_order = if follows_carrier {
                [0, 1, 2, 3]
            } else {
                [1, 0, 3, 2]
            };
            let mut directions = Vec::new();
            let fragments = source_order
                .into_iter()
                .map(|carrier_index| {
                    directions.push(direction(carrier_index, follows_carrier));
                    let source = &source_splits[carrier_index];
                    let fragment = if follows_carrier {
                        source.fragment.clone()
                    } else {
                        source.fragment.reversed().unwrap()
                    };
                    let (start, end) = if follows_carrier {
                        (source.start_topology_vertex, source.end_topology_vertex)
                    } else {
                        (source.end_topology_vertex, source.start_topology_vertex)
                    };
                    BezierArrangementFragment2::new(carrier_index, 0, fragment)
                        .with_topology_vertices(start, end)
                })
                .collect();
            let graph = BezierArrangementGraph2::from_certified_fragments(fragments);
            let starts_by_vertex = arrangement_starts_by_vertex(&graph, None);
            let mut successors = vec![None; graph.len()];
            certify_nontransverse_authored_continuity(
                &mut successors,
                &graph,
                &directions,
                &topology,
                &carriers,
                &starts_by_vertex,
            );
            assert_eq!(successors, [Some(1), None, Some(3), None]);
            let Classification::Decided(traversal) = graph
                .traverse_retained_with_certified_successors(&successors, &CurveContext::STRICT)
            else {
                panic!("the certified point-touch loops must traverse");
            };
            assert_eq!(traversal.chains().len(), 2);
            assert!(traversal.chains().iter().all(|chain| chain.is_closed()));

            let mut changed_topology = topology.clone();
            changed_topology.split_fragments[source_order[1]][0].location =
                Some(RegionPointLocation::Inside);
            let mut rejected = vec![None; graph.len()];
            certify_nontransverse_authored_continuity(
                &mut rejected,
                &graph,
                &directions,
                &changed_topology,
                &carriers,
                &starts_by_vertex,
            );
            assert_eq!(rejected[0], None);
            assert_eq!(rejected[2], Some(3));
        }
    }

    #[test]
    fn certified_branch_order_matches_exact_vector_order() {
        for crossing_is_positive in [false, true] {
            for base_carrier in [3, 7] {
                for base_forward in [false, true] {
                    for first_carrier in [3, 7] {
                        for first_forward in [false, true] {
                            for second_carrier in [3, 7] {
                                for second_forward in [false, true] {
                                    let base = direction(base_carrier, base_forward);
                                    let first = direction(first_carrier, first_forward);
                                    let second = direction(second_carrier, second_forward);
                                    assert_eq!(
                                        certified_turn_preference(
                                            contact_direction(base_carrier, base_forward),
                                            contact_direction(first_carrier, first_forward),
                                            contact_direction(second_carrier, second_forward),
                                            crossing_is_positive,
                                        ),
                                        numerical_turn_preference(
                                            vector(base, crossing_is_positive),
                                            vector(first, crossing_is_positive),
                                            vector(second, crossing_is_positive),
                                        ),
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
