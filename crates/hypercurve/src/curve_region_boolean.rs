//! Immediate exact Booleans over curved regions.

use std::cmp::Ordering;
use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

use crate::bezier_moment::RationalQuadraticAreaIntegralCache;
use crate::bezier_tangent_order::algebraic_endpoint_tangents_are_transverse;
use crate::classify::real_sign;
use crate::curve_intersection::{CurveIntersectionBatchCache, CurveIntersectionContext};
use crate::policy::resolve_certified_operation;
use crate::rational_bezier_general::RationalBezierOverlapParameterCorrespondence2;
use crate::{
    Aabb2, BezierArrangementFragment2, BezierArrangementGraph2, BezierEndpointTangentImage2,
    BezierLineContactRelation, BezierLineImageFitRelation, BezierParallel2,
    BezierParallelPairIntersectionSet2, BezierParameter2, BezierParameterRange2,
    BezierSplitFragment2, BezierSubcurve2, BooleanOp, Classification, Curve2, CurveContext,
    CurveDerivative2, CurveError, CurveFamily2, CurveIntersectionContact2,
    CurveIntersectionOverlap2, CurveIntersectionPairBlocker2, CurveOperation2, CurveOutcome,
    CurvePathBooleanOperand2, CurveRegion2, CurveResult, ExactCurveError, ExactCurveResult,
    FillRule, LineSeg2, QuadraticBezier2, RationalBezier2, RationalBezierIntersectionContacts2,
    RationalBezierIntersectionOverlap2, RationalBezierIntersectionPointEvidence2,
    RationalBezierOverlapOrientation2, RationalBezierPointIncidence2, Real, RealSign,
    RegionPointLocation, Segment2, UncertaintyReason,
};

/// Stable identity for one retained region-boundary carrier.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionCarrierRef2 {
    carrier_index: usize,
    operand: CurvePathBooleanOperand2,
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
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
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
    operand: CurvePathBooleanOperand2,
    loop_index: usize,
    fragment_index: usize,
    family: CurveFamily2,
    geometry: RegionCarrierGeometry,
    start: BezierParameter2,
    end: BezierParameter2,
    reversed: bool,
    filled_side_is_left: bool,
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
}

#[derive(Debug)]
enum RegionCarrierPairContext {
    Bezier(CurveIntersectionContext),
    ParallelRational { parallel_is_first: bool },
    ParallelPair,
    ParallelSameImage,
    ParallelSelf,
    BezierSelf,
}

#[derive(Clone, Debug, PartialEq)]
enum RegionPairContactEvidence {
    Bezier(CurveIntersectionContact2),
    Direct {
        first_parameter: BezierParameter2,
        second_parameter: BezierParameter2,
        point: Option<RationalBezierIntersectionPointEvidence2>,
        certified_transverse: bool,
        tangent_cross_sign: Option<RealSign>,
    },
}

#[derive(Clone, Debug)]
struct RegionPairOverlap {
    source: Option<CurveIntersectionOverlap2>,
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
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

#[derive(Clone, Debug)]
struct CarrierEvent {
    parameter: BezierParameter2,
    topology_vertex: Option<usize>,
}

#[derive(Clone, Debug)]
struct ContactVertex {
    point: Option<RationalBezierIntersectionPointEvidence2>,
    topology_vertex: usize,
    carrier_indices: [usize; 2],
    parameters: [BezierParameter2; 2],
}

#[derive(Clone, Debug)]
struct CarrierOverlap {
    first_carrier_index: usize,
    second_carrier_index: usize,
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
}

#[derive(Debug)]
enum CarrierOverlapClip {
    Unmatched,
    Matched(Option<(BezierParameterRange2, BezierParameterRange2)>),
}

#[derive(Clone, Debug)]
struct TransitionContactCandidate {
    first_carrier: usize,
    second_carrier: usize,
    certified_transverse: bool,
    cross_is_positive: Option<bool>,
    self_parameters: Option<[BezierParameter2; 2]>,
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
    location: RegionPointLocation,
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

impl CurveRegionCarrierRef2 {
    /// Returns the flattened carrier index in the retained pair.
    pub const fn carrier_index(&self) -> usize {
        self.carrier_index
    }

    /// Returns the region operand that owns this carrier.
    pub const fn operand(&self) -> CurvePathBooleanOperand2 {
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
    pub const fn first_parameter(&self) -> &BezierParameter2 {
        self.evidence.first_parameter()
    }

    /// Returns the exact parameter on the second retained carrier.
    pub const fn second_parameter(&self) -> &BezierParameter2 {
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
    pub const fn first_range(&self) -> &BezierParameterRange2 {
        &self.first_range
    }

    /// Returns the exact overlap range clipped to the second retained carrier.
    pub const fn second_range(&self) -> &BezierParameterRange2 {
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
    const fn first_parameter(&self) -> &BezierParameter2 {
        match self {
            Self::Bezier(contact) => contact.first().local_parameter(),
            Self::Direct {
                first_parameter, ..
            } => first_parameter,
        }
    }

    const fn second_parameter(&self) -> &BezierParameter2 {
        match self {
            Self::Bezier(contact) => contact.second().local_parameter(),
            Self::Direct {
                second_parameter, ..
            } => second_parameter,
        }
    }

    const fn point(&self) -> Option<&RationalBezierIntersectionPointEvidence2> {
        match self {
            Self::Bezier(contact) => Some(contact.point()),
            Self::Direct { point, .. } => point.as_ref(),
        }
    }

    const fn is_certified_transverse(&self) -> bool {
        match self {
            Self::Bezier(contact) => contact.is_certified_transverse(),
            Self::Direct {
                certified_transverse,
                ..
            } => *certified_transverse,
        }
    }

    const fn tangent_cross_is_positive(&self) -> Option<bool> {
        match self {
            Self::Bezier(_) => None,
            Self::Direct {
                tangent_cross_sign, ..
            } => match tangent_cross_sign {
                Some(RealSign::Positive) => Some(true),
                Some(RealSign::Negative) => Some(false),
                Some(RealSign::Zero) | None => None,
            },
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
        if self.is_empty() {
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
}

impl<'a> CurveRegionBooleanContext<'a> {
    fn try_new(
        first: &'a CurveRegion2,
        second: &'a CurveRegion2,
        policy: &'a CurveContext,
    ) -> ExactCurveResult<Self> {
        let mut rational_quadratic_area_cache = RationalQuadraticAreaIntegralCache::default();
        let first_carriers = build_region_carriers(
            first,
            CurvePathBooleanOperand2::First,
            policy,
            &mut rational_quadratic_area_cache,
            true,
        )?;
        let first_carrier_count = first_carriers.len();
        let mut carriers = first_carriers;
        carriers.extend(build_region_carriers(
            second,
            CurvePathBooleanOperand2::Second,
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
                RegionCarrierGeometry::AnalyticParallel(_) => None,
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

    fn try_new_unary(region: &'a CurveRegion2, policy: &'a CurveContext) -> ExactCurveResult<Self> {
        let mut rational_quadratic_area_cache = RationalQuadraticAreaIntegralCache::default();
        let carriers = build_region_carriers(
            region,
            CurvePathBooleanOperand2::First,
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
                RegionCarrierGeometry::AnalyticParallel(_) => None,
            })
            .collect::<Vec<_>>();
        let mut pairs = Vec::with_capacity(carrier_count.saturating_mul(2));
        let mut intersection_cache = CurveIntersectionBatchCache::default();
        for first_carrier_index in 0..carrier_count {
            for second_carrier_index in first_carrier_index + 1..carrier_count {
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
            let carrier = &carriers[first_carrier_index];
            if !carrier.geometry.has_certified_injective_axis(policy) {
                pairs.push(RegionCarrierPair {
                    first_carrier_index,
                    second_carrier_index: first_carrier_index,
                    context: match &carrier.geometry {
                        RegionCarrierGeometry::AnalyticParallel(_) => {
                            RegionCarrierPairContext::ParallelSelf
                        }
                        RegionCarrierGeometry::Bezier(_) => RegionCarrierPairContext::BezierSelf,
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
                    source: overlap.source,
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

    fn parallel_line_pair_result(
        &self,
        parallel: &BezierParallel2,
        curve: &BezierSubcurve2,
        parallel_is_first: bool,
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
        let relation = match parallel
            .relation_to_supporting_line_with_contacts(&line, &self.data.policy)
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
        for contact in contacts {
            let from_start = match parallel
                .supporting_line_parameter_order(contact.parameter(), &line, &self.data.policy)
                .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let from_end = match parallel
                .supporting_line_parameter_order(
                    contact.parameter(),
                    &reversed_line,
                    &self.data.policy,
                )
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
        self.parallel_exact_parameter_pair_result(
            parallel,
            curve,
            retained_parameters,
            parallel_is_first,
        )
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
        let incidence = match parallel
            .circle_incidence(arc.center(), arc.radius_squared_ref(), &self.data.policy)
            .map_err(|cause| self.invalid(0, cause))?
        {
            Classification::Decided(incidence) => incidence,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let parameters = match incidence {
            crate::BezierParallelIncidence2::EntireCurve => {
                return Ok(Classification::Decided(None));
            }
            crate::BezierParallelIncidence2::Parameters(parameters) => parameters,
        };
        let mut retained_parameters = Vec::with_capacity(parameters.len());
        for parameter in parameters {
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
                result_contacts.push(RegionPairContactEvidence::Direct {
                    first_parameter,
                    second_parameter,
                    point: Some(RationalBezierIntersectionPointEvidence2::Exact(
                        point.clone(),
                    )),
                    certified_transverse: tangent_cross_sign.is_some(),
                    tangent_cross_sign,
                });
            }
        }
        Ok(Classification::Decided(Some(RegionPairResult {
            contacts: result_contacts,
            overlaps: Vec::new(),
            blockers: Vec::new(),
        })))
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
                        .cloned()
                        .map(RegionPairContactEvidence::Bezier)
                        .collect(),
                    overlaps: result
                        .overlaps()
                        .iter()
                        .cloned()
                        .map(|source| RegionPairOverlap {
                            first_range: source.first_range().clone(),
                            second_range: source.second_range().clone(),
                            orientation: source.orientation(),
                            source: Some(source),
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
                    RegionPairContactEvidence::Direct {
                        first_parameter: contact.first_parameter().clone(),
                        second_parameter: contact.second_parameter().clone(),
                        point: Some(contact.point().clone()),
                        certified_transverse: contact.is_certified_transverse(),
                        tangent_cross_sign: contact.tangent_cross_sign(),
                    }
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
                let (parallel, curve) = if *parallel_is_first {
                    (first.geometry.parallel(), second.geometry.bezier())
                } else {
                    (second.geometry.parallel(), first.geometry.bezier())
                };
                match self.parallel_arc_pair_result(parallel, curve, *parallel_is_first)? {
                    Classification::Decided(Some(result)) => return Ok(result),
                    Classification::Decided(None) | Classification::Uncertain(_) => {}
                }
                match self.parallel_line_pair_result(parallel, curve, *parallel_is_first)? {
                    Classification::Decided(Some(result)) => return Ok(result),
                    Classification::Decided(None) | Classification::Uncertain(_) => {}
                }
                let rational = RationalBezier2::try_from_subcurve(curve)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
                let result = match parallel
                    .intersections(&rational, &self.data.policy)
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
                        RegionPairContactEvidence::Direct {
                            first_parameter,
                            second_parameter,
                            point: Some(contact.point().clone()),
                            certified_transverse: contact.is_certified_transverse(),
                            tangent_cross_sign,
                        }
                    })
                    .collect();
                let overlaps = result
                    .overlaps()
                    .iter()
                    .map(|overlap| {
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
                        RegionPairOverlap {
                            source: None,
                            first_range,
                            second_range,
                            orientation: overlap.orientation(),
                        }
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
                let parallel = first.geometry.parallel();
                let mut intersections = Vec::with_capacity(2);
                match &pair.context {
                    RegionCarrierPairContext::ParallelPair => intersections.push((
                        parallel
                            .parallel_intersections(second.geometry.parallel(), &self.data.policy),
                        false,
                    )),
                    RegionCarrierPairContext::ParallelSameImage => {
                        intersections.push((
                            parallel.parallel_intersections(
                                second.geometry.parallel(),
                                &self.data.policy,
                            ),
                            false,
                        ));
                        intersections.push((self.parallel_self_intersections(parallel), true));
                    }
                    RegionCarrierPairContext::ParallelSelf => {
                        intersections.push((self.parallel_self_intersections(parallel), true));
                    }
                    RegionCarrierPairContext::Bezier(_)
                    | RegionCarrierPairContext::BezierSelf
                    | RegionCarrierPairContext::ParallelRational { .. } => unreachable!(),
                }
                let mut contacts = Vec::new();
                let mut overlaps = Vec::new();
                let mut blockers = Vec::new();
                for (intersection, self_contacts) in intersections {
                    let result = match intersection
                        .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                    {
                        Classification::Decided(result) => result,
                        Classification::Uncertain(reason) => {
                            blockers.push(RegionPairBlocker::Uncertain(reason));
                            continue;
                        }
                    };
                    for contact in result.contacts() {
                        let (mut first_parameter, mut second_parameter) = (
                            contact.first_parameter().clone(),
                            contact.second_parameter().clone(),
                        );
                        let mut tangent_cross_sign = contact.tangent_cross_sign();
                        if self_contacts
                            && pair.first_carrier_index != pair.second_carrier_index
                            && !(parameter_in_carrier(&first_parameter, first, &self.data.policy)?
                                && parameter_in_carrier(
                                    &second_parameter,
                                    second,
                                    &self.data.policy,
                                )?)
                        {
                            std::mem::swap(&mut first_parameter, &mut second_parameter);
                            tangent_cross_sign = tangent_cross_sign.map(|sign| match sign {
                                RealSign::Positive => RealSign::Negative,
                                RealSign::Negative => RealSign::Positive,
                                RealSign::Zero => RealSign::Zero,
                            });
                        }
                        contacts.push(RegionPairContactEvidence::Direct {
                            first_parameter,
                            second_parameter,
                            point: None,
                            certified_transverse: contact.is_certified_transverse(),
                            tangent_cross_sign,
                        });
                    }
                    overlaps.extend(result.overlaps().iter().map(|overlap| RegionPairOverlap {
                        source: None,
                        first_range: overlap.first_range().clone(),
                        second_range: overlap.second_range().clone(),
                        orientation: overlap.orientation(),
                    }));
                    if !result.parameter_components().is_empty() {
                        blockers.push(RegionPairBlocker::PointImageParameterComponent);
                    }
                    if !result.is_complete() {
                        blockers.push(RegionPairBlocker::IncompleteReplay);
                    }
                }
                Ok(RegionPairResult {
                    contacts,
                    overlaps,
                    blockers,
                })
            }
        }
    }

    fn clipped_overlap_ranges(
        &self,
        pair: &RegionCarrierPair,
        overlap: &RegionPairOverlap,
    ) -> ExactCurveResult<Option<(BezierParameterRange2, BezierParameterRange2)>> {
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
        let correspondence = overlap
            .source
            .as_ref()
            .and_then(CurveIntersectionOverlap2::parameter_correspondence);
        if let Some(correspondence) = correspondence {
            if let Some(reversed) = correspondence.projective_reversal() {
                return match clip_aligned_parameter_overlap(
                    &overlap.first_range,
                    &overlap.second_range,
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
                &overlap.first_range,
                &overlap.second_range,
                correspondence,
                first_carrier,
                second_carrier,
                &self.data.policy,
            );
        }
        match clip_projectively_aligned_parameter_overlap(
            &overlap.first_range,
            &overlap.second_range,
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
                    overlap.first_range.start().clone(),
                    overlap.first_range.end().clone(),
                    overlap.second_range.start().clone(),
                    overlap.second_range.end().clone(),
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
                    &overlap.first_range,
                    &overlap.second_range,
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

    fn build_split_topology(&self) -> ExactCurveResult<CurveRegionSplitTopology> {
        let mut events = vec![Vec::new(); self.data.carriers.len()];
        let mut contact_points = Vec::<ContactVertex>::new();
        let mut next_topology_vertex = 0_usize;
        let mut contact_vertex_counts = Vec::<usize>::new();
        let mut transition_candidates = Vec::<Option<TransitionContactCandidate>>::new();
        let mut reclassification_vertices = Vec::<bool>::new();
        seed_loop_topology_vertices(
            &self.data.carriers,
            &mut events,
            &mut next_topology_vertex,
            &self.data.policy,
        )?;
        contact_vertex_counts.resize(next_topology_vertex, 0);
        transition_candidates.resize(next_topology_vertex, None);
        reclassification_vertices.resize(next_topology_vertex, false);
        let mut overlaps = Vec::new();
        for pair in &self.data.pairs {
            let result = self.pair_result(pair)?;
            if let Some(blocker) = result.blockers.first() {
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
                if !parameter_in_carrier(
                    first_parameter,
                    &self.data.carriers[pair.first_carrier_index],
                    &self.data.policy,
                )? || !parameter_in_carrier(
                    second_parameter,
                    &self.data.carriers[pair.second_carrier_index],
                    &self.data.policy,
                )? {
                    continue;
                }
                let first_existing = existing_event_vertex(
                    &events[pair.first_carrier_index],
                    first_parameter,
                    &self.data.policy,
                )?;
                let second_existing = existing_event_vertex(
                    &events[pair.second_carrier_index],
                    second_parameter,
                    &self.data.policy,
                )?;
                let mut matching_contact_index = None;
                for (existing_index, existing) in contact_points.iter().enumerate() {
                    if contacts_decided_distinct_from_carriers(
                        existing,
                        [pair.first_carrier_index, pair.second_carrier_index],
                        [first_parameter, second_parameter],
                        &self.data.carriers,
                        &self.data.policy,
                    )? {
                        continue;
                    }
                    if let (Some(existing_point), Some(point)) =
                        (existing.point.as_ref(), contact.point())
                    {
                        match same_contact_point(existing_point, point, &self.data.policy) {
                            Classification::Decided(true) => {
                                matching_contact_index = Some(existing_index);
                                break;
                            }
                            Classification::Decided(false) => {}
                            Classification::Uncertain(reason) => {
                                return Err(self.blocked(pair.first_carrier_index, reason));
                            }
                        }
                    }
                }
                let matching_contact_vertex =
                    matching_contact_index.map(|index| contact_points[index].topology_vertex);
                let topology_vertex = first_existing
                    .or(second_existing)
                    .or(matching_contact_vertex)
                    .unwrap_or_else(|| {
                        let vertex = next_topology_vertex;
                        next_topology_vertex += 1;
                        vertex
                    });
                for previous_vertex in [first_existing, second_existing, matching_contact_vertex]
                    .into_iter()
                    .flatten()
                    .filter(|previous| *previous != topology_vertex)
                {
                    replace_topology_vertex(
                        &mut events,
                        &mut contact_points,
                        previous_vertex,
                        topology_vertex,
                    );
                    contact_vertex_counts[topology_vertex] +=
                        contact_vertex_counts[previous_vertex];
                    contact_vertex_counts[previous_vertex] = 0;
                    reclassification_vertices[topology_vertex] |=
                        reclassification_vertices[previous_vertex];
                    reclassification_vertices[previous_vertex] = false;
                    transition_candidates[topology_vertex] = None;
                    transition_candidates[previous_vertex] = None;
                }
                if let Some(index) = matching_contact_index {
                    if matches!(
                        contact_points[index].point,
                        Some(RationalBezierIntersectionPointEvidence2::Algebraic(_))
                    ) && matches!(
                        contact.point(),
                        Some(RationalBezierIntersectionPointEvidence2::Exact(_))
                    ) {
                        contact_points[index].point = contact.point().cloned();
                    }
                } else {
                    contact_points.push(ContactVertex {
                        point: contact.point().cloned(),
                        topology_vertex,
                        carrier_indices: [pair.first_carrier_index, pair.second_carrier_index],
                        parameters: [first_parameter.clone(), second_parameter.clone()],
                    });
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
                    && parameter_strictly_inside_carrier(
                        first_parameter,
                        &self.data.carriers[pair.first_carrier_index],
                        &self.data.policy,
                    )
                    && parameter_strictly_inside_carrier(
                        second_parameter,
                        &self.data.carriers[pair.second_carrier_index],
                        &self.data.policy,
                    ) {
                    Some(TransitionContactCandidate {
                        first_carrier: pair.first_carrier_index,
                        second_carrier: pair.second_carrier_index,
                        certified_transverse: contact.is_certified_transverse(),
                        cross_is_positive: contact.tangent_cross_is_positive(),
                        self_parameters: (pair.first_carrier_index == pair.second_carrier_index)
                            .then(|| [first_parameter.clone(), second_parameter.clone()]),
                    })
                } else {
                    None
                };
                push_carrier_event(
                    &mut events[pair.first_carrier_index],
                    first_parameter.clone(),
                    Some(topology_vertex),
                    &self.data.policy,
                )?;
                push_carrier_event(
                    &mut events[pair.second_carrier_index],
                    second_parameter.clone(),
                    Some(topology_vertex),
                    &self.data.policy,
                )?;
            }

            for overlap in &result.overlaps {
                let Some((first_range, second_range)) =
                    self.clipped_overlap_ranges(pair, overlap)?
                else {
                    continue;
                };
                let first_parameters = [first_range.start(), first_range.end()];
                let second_parameters = [second_range.start(), second_range.end()];
                for (parameter, second_parameter) in
                    first_parameters.into_iter().zip(second_parameters)
                {
                    push_carrier_event(
                        &mut events[pair.first_carrier_index],
                        parameter.clone(),
                        None,
                        &self.data.policy,
                    )?;
                    push_carrier_event(
                        &mut events[pair.second_carrier_index],
                        second_parameter.clone(),
                        None,
                        &self.data.policy,
                    )?;
                }
                overlaps.push(CarrierOverlap {
                    first_carrier_index: pair.first_carrier_index,
                    second_carrier_index: pair.second_carrier_index,
                    first_range,
                    second_range,
                    orientation: overlap.orientation,
                });
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
                        && let Some(candidate) = transition_candidates.get_mut(vertex)
                    {
                        *candidate = None;
                        reclassification_vertices[vertex] = true;
                    }
                }
            }
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
            &transition_candidates,
            &self.data.policy,
        );
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
            transverse_contacts,
            transverse_vertices,
            reclassification_vertices,
        })
    }

    fn build_boolean_topology(&self) -> ExactCurveResult<CurveRegionBooleanTopology> {
        let CurveRegionSplitTopology {
            split_fragments,
            overlaps,
            transverse_contacts,
            transverse_vertices,
            reclassification_vertices,
        } = self.build_split_topology()?;
        let mut classified_split_fragments = Vec::with_capacity(split_fragments.len());
        let mut point_classification_count = 0_usize;
        let mut previous = None::<(
            CurvePathBooleanOperand2,
            usize,
            Option<usize>,
            RegionPointLocation,
        )>;
        for (carrier_index, fragments) in split_fragments.into_iter().enumerate() {
            let carrier = &self.data.carriers[carrier_index];
            let mut classified =
                Vec::<ClassifiedSplitCarrierFragment>::with_capacity(fragments.len());
            for split in fragments {
                let propagated =
                    previous.and_then(|(operand, loop_index, end_topology_vertex, location)| {
                        (operand == carrier.operand
                            && loop_index == carrier.loop_index
                            && end_topology_vertex == split.start_topology_vertex)
                            .then_some(split.start_topology_vertex)
                            .flatten()
                            .and_then(|vertex| {
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
                            })
                    });
                let location = match propagated {
                    Some(location) => location,
                    None => {
                        let (start, end) = fragment_range(&split.fragment);
                        let mut shared = false;
                        for overlap in &overlaps {
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
                                shared = true;
                                break;
                            }
                        }
                        if shared {
                            RegionPointLocation::Boundary
                        } else {
                            point_classification_count += 1;
                            self.fragment_location(carrier_index, &split.fragment)?
                        }
                    }
                };
                previous = Some((
                    carrier.operand,
                    carrier.loop_index,
                    split.end_topology_vertex,
                    location,
                ));
                classified.push(ClassifiedSplitCarrierFragment { split, location });
            }
            classified_split_fragments.push(classified);
        }
        Ok(CurveRegionBooleanTopology {
            split_fragments: classified_split_fragments,
            overlaps,
            transverse_contacts,
            point_classification_count,
        })
    }

    fn build_boolean_regions(&self) -> ExactCurveResult<CurveRegionBooleanResults2> {
        let operations = [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ];
        let topology = self.build_boolean_topology()?;
        let regions = [
            self.build_boolean_region_from_topology(operations[0], &topology)?,
            self.build_boolean_region_from_topology(operations[1], &topology)?,
            self.build_boolean_region_from_topology(operations[2], &topology)?,
            match self.build_boolean_region_from_topology(operations[3], &topology) {
                Ok(region) => region,
                Err(ExactCurveError::Blocked(_)) => self.build_xor_from_exact_set_identity()?,
                Err(error) => return Err(error),
            },
        ];
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
        parameter: &BezierParameter2,
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
        let mut arrangement_fragments = Vec::new();
        let mut arrangement_directions = Vec::new();
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for (split_fragment_index, split) in splits.iter().enumerate() {
                let (source_start, source_end) = split.fragment.parameter_range();
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
                let action = self.regularized_fragment_action(
                    carrier_index,
                    &split.fragment,
                    &topology.overlaps,
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
        let certified_successors = certified_regularization_successors(
            &graph,
            &arrangement_directions,
            &topology.transverse_contacts,
        );
        let traversal = match graph.traverse_retained_filled_left_faces_with_certified_successors(
            &certified_successors,
            &self.data.policy,
        ) {
            Classification::Decided(traversal) => traversal,
            Classification::Uncertain(reason) => return Err(self.blocked(0, reason)),
        };
        let region =
            match CurveRegion2::from_certified_retained_arrangement_traversal(&graph, &traversal) {
                Classification::Decided(region) => region,
                Classification::Uncertain(reason) => return Err(self.blocked(0, reason)),
            }
            .with_certified_filled_side_is_left(vec![true; traversal.chains().len()])
            .map_err(|cause| self.invalid(0, cause))?;
        if affine_line_output || self.strict_line_image_only() {
            self.compact_line_image_result(region)
        } else {
            Ok(region)
        }
    }

    fn regularized_fragment_action(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        overlaps: &[CarrierOverlap],
    ) -> ExactCurveResult<RegionFragmentAction> {
        if !self.regularized_fragment_owns_overlap(carrier_index, fragment, overlaps)? {
            return Ok(RegionFragmentAction::Discard);
        }
        let (parameter, representative) = self.fragment_representative(carrier_index, fragment)?;
        let carrier = &self.data.carriers[carrier_index];
        let derivative = match carrier
            .geometry
            .derivative_at(&parameter, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(derivative) => derivative,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        let (mut tangent_x, mut tangent_y) = (derivative.dx().clone(), derivative.dy().clone());
        if carrier.reversed {
            tangent_x = -tangent_x;
            tangent_y = -tangent_y;
        }
        let tangent_squared = &tangent_x * &tangent_x + &tangent_y * &tangent_y;
        match crate::classify::is_zero(&tangent_squared, &self.data.policy) {
            Some(false) => {}
            Some(true) => return Err(self.blocked(carrier_index, UncertaintyReason::Boundary)),
            None => return Err(self.blocked(carrier_index, UncertaintyReason::RealSign)),
        }
        let source_parameter = BezierParameter2::Exact(parameter);
        let left = self.fragment_side_location(
            carrier_index,
            &representative,
            &source_parameter,
            &tangent_x,
            &tangent_y,
            true,
        )?;
        let right = self.fragment_side_location(
            carrier_index,
            &representative,
            &source_parameter,
            &tangent_x,
            &tangent_y,
            false,
        )?;
        Ok(action_from_result_sides(
            left == RegionPointLocation::Inside,
            right == RegionPointLocation::Inside,
        ))
    }

    fn regularized_fragment_owns_overlap(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        overlaps: &[CarrierOverlap],
    ) -> ExactCurveResult<bool> {
        let (start, end) = fragment_range(fragment);
        for overlap in overlaps {
            let (own_range, other_carrier_index) = if overlap.first_carrier_index == carrier_index {
                (&overlap.first_range, overlap.second_carrier_index)
            } else if overlap.second_carrier_index == carrier_index {
                (&overlap.second_range, overlap.first_carrier_index)
            } else {
                continue;
            };
            if other_carrier_index < carrier_index
                && range_contains_fragment(own_range, start, end, &self.data.policy)?
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn fragment_side_location(
        &self,
        carrier_index: usize,
        representative: &crate::Point2,
        source_parameter: &BezierParameter2,
        tangent_x: &crate::Real,
        tangent_y: &crate::Real,
        left: bool,
    ) -> ExactCurveResult<RegionPointLocation> {
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
                .classify_point_from_boundary_side_ray(
                    representative,
                    direction_x,
                    direction_y,
                    carrier.loop_index,
                    carrier.fragment_index,
                    source_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(location) => return Ok(location),
                Classification::Uncertain(reason) => last_reason = reason,
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
            .into_boundary_loops()
            .into_iter()
            // Both derived regions reuse the operands' source records. Strip
            // those records before combining them into one independent region.
            .map(crate::CurveRegionBoundaryLoop2::without_arrangement_sources)
            .collect::<Vec<_>>();
        union_loops.extend(
            intersection
                .into_boundary_loops()
                .into_iter()
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
                    classified.location,
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
            .with_certified_filled_side_is_left(vec![true; traversal.chains().len()])
            .map_err(|cause| self.invalid(0, cause))?;
        if affine_line_output || self.strict_line_image_only() {
            self.compact_line_image_result(region)
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
                    RegionCarrierGeometry::AnalyticParallel(_) => false,
                })
        })
    }

    fn compact_line_image_result(&self, region: CurveRegion2) -> ExactCurveResult<CurveRegion2> {
        if region.is_empty() {
            return Ok(region);
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
            return match mixed_roles {
                Some(roles) => region.with_certified_loop_roles(roles),
                None => region.with_certified_all_material_loop_roles(loop_count),
            }
            .map_err(|cause| self.invalid(0, cause));
        }
        CurveRegion2::from_certified_oriented_line_contours(material, holes)
            .map_err(|cause| self.invalid(0, cause))
    }

    fn fragment_location(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
    ) -> ExactCurveResult<RegionPointLocation> {
        let (_, representative) = self.fragment_representative(carrier_index, fragment)?;
        let carrier = &self.data.carriers[carrier_index];
        let other = match carrier.operand {
            CurvePathBooleanOperand2::First => &self.data.second,
            CurvePathBooleanOperand2::Second => &self.data.first,
        };
        match other
            .classify_point_raw(&representative, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(location) => Ok(location),
            Classification::Uncertain(reason) => Err(self.blocked(carrier_index, reason)),
        }
    }

    fn fragment_representative(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
    ) -> ExactCurveResult<(crate::Real, crate::Point2)> {
        let carrier = &self.data.carriers[carrier_index];
        let (start, end) = fragment_range(fragment);
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
        let (start, end) = fragment_range(fragment);
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
        Ok(action_from_result_sides(
            operation.apply(first.filled_side_is_left, second_left_in_first_direction),
            operation.apply(!first.filled_side_is_left, !second_left_in_first_direction),
        ))
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
        (RegionCarrierGeometry::Bezier(_), RegionCarrierGeometry::Bezier(_)) => {
            RegionCarrierPairContext::Bezier(CurveIntersectionContext::try_new_with_batch_cache(
                curves[first_carrier_index]
                    .as_ref()
                    .expect("Bezier carrier has a top-level curve"),
                curves[second_carrier_index]
                    .as_ref()
                    .expect("Bezier carrier has a top-level curve"),
                policy,
                intersection_cache,
            )?)
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
    matches!(
        fragment,
        BezierSplitFragment2::Materialized {
            curve: BezierSubcurve2::Quadratic(curve),
            ..
        } if curve.retained_exact_line_image().is_some()
    )
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
}

fn subcurve_has_certified_injective_axis(curve: &BezierSubcurve2, policy: &CurveContext) -> bool {
    let rational = match curve {
        BezierSubcurve2::Quadratic(curve) => RationalBezier2::try_new(
            curve.control_points().into_iter().cloned().collect(),
            vec![crate::Real::one(); 3],
        ),
        BezierSubcurve2::Cubic(curve) => RationalBezier2::try_new(
            curve.control_points().into_iter().cloned().collect(),
            vec![crate::Real::one(); 4],
        ),
        BezierSubcurve2::RationalQuadratic(curve) => RationalBezier2::try_new(
            curve.control_points().into_iter().cloned().collect(),
            curve.weights().into_iter().cloned().collect(),
        ),
        BezierSubcurve2::Rational(curve) => return curve.has_certified_injective_axis(policy),
    };
    rational.is_ok_and(|curve| curve.has_certified_injective_axis(policy))
}

fn subcurve_certified_outer_bounds(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.control_hull_box(policy),
        BezierSubcurve2::Cubic(curve) => curve.control_hull_box(policy),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(policy),
    }
}

fn build_region_carriers(
    region: &CurveRegion2,
    operand: CurvePathBooleanOperand2,
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
            let (mut geometry, mut start, mut end, mut reversed) = match fragment {
                BezierSplitFragment2::Materialized { curve, .. } => (
                    RegionCarrierGeometry::Bezier(curve.clone()),
                    BezierParameter2::Exact(crate::Real::zero()),
                    BezierParameter2::Exact(crate::Real::one()),
                    false,
                ),
                BezierSplitFragment2::AlgebraicEndpointImages {
                    reversed,
                    start,
                    end,
                    source_curve: Some(curve),
                    ..
                } => (
                    RegionCarrierGeometry::Bezier(curve.clone()),
                    start.clone(),
                    end.clone(),
                    *reversed,
                ),
                BezierSplitFragment2::AlgebraicEndpointImages {
                    source_curve: None, ..
                }
                | BezierSplitFragment2::Unresolved { .. } => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Boolean,
                        CurveFamily2::RationalBezier,
                        UncertaintyReason::Unsupported,
                    ));
                }
                BezierSplitFragment2::AnalyticParallel(fragment) => (
                    RegionCarrierGeometry::AnalyticParallel(fragment.parallel().clone()),
                    fragment.range().start().clone(),
                    fragment.range().end().clone(),
                    fragment.is_reversed(),
                ),
            };
            let family = geometry.family();
            if matches!(
                fragment,
                BezierSplitFragment2::AlgebraicEndpointImages { .. }
            ) && let Ok(Classification::Decided(line)) =
                crate::bezier_region::retained_line_fragment_segment(fragment, policy)
            {
                geometry = RegionCarrierGeometry::Bezier(BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(line),
                ));
                start = BezierParameter2::Exact(crate::Real::zero());
                end = BezierParameter2::Exact(crate::Real::one());
                reversed = false;
            }
            carriers.push(RegionCarrier {
                operand,
                loop_index,
                fragment_index,
                family,
                geometry,
                start,
                end,
                reversed,
                filled_side_is_left: filled_sides[loop_index],
                image_is_injective: OnceLock::new(),
                bounds: OnceLock::new(),
            });
        }
    }
    Ok(carriers)
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
    for max_refinement_steps in [1, 2, 4] {
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
    if let RegionCarrierGeometry::AnalyticParallel(parallel) = &carrier.geometry {
        return split_analytic_carrier(carrier, parallel, events, max_refinement_steps, policy);
    }
    let parameters = events
        .iter()
        .map(|event| {
            event
                .parameter
                .clone()
                .refined_isolating_interval(max_refinement_steps, policy)
        })
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
        let (start, end) = fragment_range(fragment);
        if !parameter_range_inside_carrier(start, end, carrier, policy)? {
            continue;
        }
        let start_topology_vertex = event_vertex(events, start, policy)?;
        let end_topology_vertex = event_vertex(events, end, policy)?;
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

fn split_analytic_carrier(
    carrier: &RegionCarrier,
    parallel: &BezierParallel2,
    events: &[CarrierEvent],
    max_refinement_steps: usize,
    policy: &CurveContext,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
    let mut boundaries = events
        .iter()
        .map(|event| CarrierEvent {
            parameter: event
                .parameter
                .clone()
                .refined_isolating_interval(max_refinement_steps, policy),
            topology_vertex: event.topology_vertex,
        })
        .collect::<Vec<_>>();
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
        let start = pair[0].parameter.clone();
        let end = pair[1].parameter.clone();
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
        if !parameter_range_inside_carrier(&start, &end, carrier, policy)? {
            continue;
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
    certified_transverse_successors(
        graph,
        directions,
        &topology.transverse_contacts,
        |contact, vertex| transverse_carrier_cross_is_positive(topology, contact, vertex, carriers),
    )
}

fn certified_regularization_successors(
    graph: &BezierArrangementGraph2,
    directions: &[BooleanArrangementFragmentDirection],
    contacts: &HashMap<usize, TransitionContactCandidate>,
) -> Vec<Option<usize>> {
    certified_transverse_successors(graph, directions, contacts, |contact, _| {
        contact.cross_is_positive
    })
}

fn certified_transverse_successors(
    graph: &BezierArrangementGraph2,
    directions: &[BooleanArrangementFragmentDirection],
    contacts: &HashMap<usize, TransitionContactCandidate>,
    mut crossing_is_positive: impl FnMut(&TransitionContactCandidate, usize) -> Option<bool>,
) -> Vec<Option<usize>> {
    // Index starts once so retaining branch certificates stays linear in the
    // emitted graph size even for large curved regions.
    let mut starts_by_vertex = HashMap::<usize, Vec<usize>>::new();
    for (fragment_index, fragment) in graph.fragments().iter().enumerate() {
        if let Some(vertex) = fragment.start_topology_vertex()
            && contacts.contains_key(&vertex)
        {
            starts_by_vertex
                .entry(vertex)
                .or_default()
                .push(fragment_index);
        }
    }
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
        .location;
    let after = fragments
        .iter()
        .find(|fragment| fragment.split.start_topology_vertex == Some(vertex))?
        .location;
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
    candidates: &[Option<TransitionContactCandidate>],
    policy: &CurveContext,
) -> Vec<bool> {
    candidates
        .iter()
        .enumerate()
        .map(|(vertex, candidate)| {
            let Some(candidate) = candidate else {
                return false;
            };
            if candidate.certified_transverse {
                return true;
            }
            let Some(first) = algebraic_endpoint_tangent_at_vertex(
                &split_fragments[candidate.first_carrier],
                vertex,
            ) else {
                return false;
            };
            let Some(second) = algebraic_endpoint_tangent_at_vertex(
                &split_fragments[candidate.second_carrier],
                vertex,
            ) else {
                return false;
            };
            matches!(
                algebraic_endpoint_tangents_are_transverse(first, second, policy),
                Classification::Decided(true)
            )
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

fn push_carrier_event(
    events: &mut Vec<CarrierEvent>,
    parameter: BezierParameter2,
    topology_vertex: Option<usize>,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    for event in events.iter_mut() {
        match parameter
            .cmp_by_refinement(&event.parameter, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Boolean,
                    CurveFamily2::RationalBezier,
                    cause,
                )
            })? {
            Classification::Decided(Ordering::Equal) => {
                if event.topology_vertex.is_none() {
                    event.topology_vertex = topology_vertex;
                }
                return Ok(());
            }
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Boolean,
                    CurveFamily2::RationalBezier,
                    reason,
                ));
            }
        }
    }
    events.push(CarrierEvent {
        parameter,
        topology_vertex,
    });
    Ok(())
}

fn seed_loop_topology_vertices(
    carriers: &[RegionCarrier],
    events: &mut [Vec<CarrierEvent>],
    next_topology_vertex: &mut usize,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
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
            push_carrier_event(
                &mut events[current_index],
                carrier_traversal_end(&carriers[current_index]).clone(),
                Some(vertex),
                policy,
            )?;
            push_carrier_event(
                &mut events[next_index],
                carrier_traversal_start(&carriers[next_index]).clone(),
                Some(vertex),
                policy,
            )?;
        }
        loop_start = loop_end;
    }
    Ok(())
}

fn carrier_traversal_start(carrier: &RegionCarrier) -> &BezierParameter2 {
    if carrier.reversed {
        &carrier.end
    } else {
        &carrier.start
    }
}

fn carrier_traversal_end(carrier: &RegionCarrier) -> &BezierParameter2 {
    if carrier.reversed {
        &carrier.start
    } else {
        &carrier.end
    }
}

fn existing_event_vertex(
    events: &[CarrierEvent],
    parameter: &BezierParameter2,
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
    parameters: [&BezierParameter2; 2],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
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
        let image_is_injective = carrier.image_is_injective.get() == Some(&true)
            || carrier.geometry.has_certified_injective_axis(policy);
        if !image_is_injective {
            continue;
        }
        let _ = carrier.image_is_injective.set(true);
        for (current_slot, current_carrier) in carrier_indices.iter().copied().enumerate() {
            if existing_carrier == current_carrier
                && decided_parameter_cmp(
                    &existing.parameters[existing_slot],
                    parameters[current_slot],
                    policy,
                )? != Ordering::Equal
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn event_vertex(
    events: &[CarrierEvent],
    parameter: &BezierParameter2,
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
    operand: CurvePathBooleanOperand2,
    own_left: bool,
    other_inside: bool,
) -> RegionFragmentAction {
    let (result_left, result_right) = match operand {
        CurvePathBooleanOperand2::First => (
            operation.apply(own_left, other_inside),
            operation.apply(!own_left, other_inside),
        ),
        CurvePathBooleanOperand2::Second => (
            operation.apply(other_inside, own_left),
            operation.apply(other_inside, !own_left),
        ),
    };
    action_from_result_sides(result_left, result_right)
}

const fn action_from_result_sides(left: bool, right: bool) -> RegionFragmentAction {
    match (left, right) {
        (true, false) => RegionFragmentAction::Keep,
        (false, true) => RegionFragmentAction::KeepReversed,
        (false, false) | (true, true) => RegionFragmentAction::Discard,
    }
}

fn parameter_in_carrier(
    parameter: &BezierParameter2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    parameter_between(parameter, &carrier.start, &carrier.end, policy)
}

fn parameter_strictly_inside_carrier(
    parameter: &BezierParameter2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> bool {
    matches!(
        (
            decided_parameter_cmp(parameter, &carrier.start, policy),
            decided_parameter_cmp(parameter, &carrier.end, policy),
        ),
        (Ok(Ordering::Greater), Ok(Ordering::Less))
    )
}

fn parameter_between(
    parameter: &BezierParameter2,
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let lower = decided_parameter_cmp(parameter, start, policy)?;
    let upper = decided_parameter_cmp(parameter, end, policy)?;
    Ok(!lower.is_lt() && !upper.is_gt())
}

fn parameter_range_inside_carrier(
    start: &BezierParameter2,
    end: &BezierParameter2,
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
    range: &BezierParameterRange2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (start, end) = ascending_range(range, policy)?;
    Ok(!decided_parameter_cmp(end, &carrier.start, policy)?.is_lt()
        && !decided_parameter_cmp(start, &carrier.end, policy)?.is_gt())
}

fn range_inside_carrier(
    range: &BezierParameterRange2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (start, end) = ascending_range(range, policy)?;
    Ok(
        !decided_parameter_cmp(start, &carrier.start, policy)?.is_lt()
            && !decided_parameter_cmp(end, &carrier.end, policy)?.is_gt(),
    )
}

fn clip_corresponding_parameter_overlap(
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    correspondence: &RationalBezierOverlapParameterCorrespondence2,
    first_carrier: &RegionCarrier,
    second_carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<Option<(BezierParameterRange2, BezierParameterRange2)>> {
    let (first_overlap_start, first_overlap_end) = ascending_range(first_range, policy)?;
    let first_start = maximum_parameter([first_overlap_start, &first_carrier.start], policy)?;
    let first_end = minimum_parameter([first_overlap_end, &first_carrier.end], policy)?;
    match decided_parameter_cmp(&first_start, &first_end, policy)? {
        Ordering::Less => {}
        Ordering::Equal | Ordering::Greater => return Ok(None),
    }
    let mapped_start = mapped_overlap_parameter(
        correspondence,
        true,
        &first_start,
        first_range,
        second_range,
        first_carrier.family,
        policy,
    )?;
    let mapped_end = mapped_overlap_parameter(
        correspondence,
        true,
        &first_end,
        first_range,
        second_range,
        first_carrier.family,
        policy,
    )?;
    let mapped_order = decided_parameter_cmp(&mapped_start, &mapped_end, policy)?;
    let (mapped_low, mapped_high) = match mapped_order {
        Ordering::Less => (&mapped_start, &mapped_end),
        Ordering::Greater => (&mapped_end, &mapped_start),
        Ordering::Equal => return Ok(None),
    };
    let (second_overlap_start, second_overlap_end) = ascending_range(second_range, policy)?;
    let second_low = maximum_parameter(
        [mapped_low, second_overlap_start, &second_carrier.start],
        policy,
    )?;
    let second_high = minimum_parameter(
        [mapped_high, second_overlap_end, &second_carrier.end],
        policy,
    )?;
    match decided_parameter_cmp(&second_low, &second_high, policy)? {
        Ordering::Less => {}
        Ordering::Equal | Ordering::Greater => return Ok(None),
    }
    if decided_parameter_cmp(&second_low, mapped_low, policy)? == Ordering::Equal
        && decided_parameter_cmp(&second_high, mapped_high, policy)? == Ordering::Equal
    {
        return Ok(Some((
            BezierParameterRange2::new_validated(first_start, first_end),
            BezierParameterRange2::new_validated(mapped_start, mapped_end),
        )));
    }
    let (second_start, second_end) = if mapped_order == Ordering::Less {
        (second_low, second_high)
    } else {
        (second_high, second_low)
    };
    let first_start = mapped_overlap_parameter(
        correspondence,
        false,
        &second_start,
        first_range,
        second_range,
        second_carrier.family,
        policy,
    )?;
    let first_end = mapped_overlap_parameter(
        correspondence,
        false,
        &second_end,
        first_range,
        second_range,
        second_carrier.family,
        policy,
    )?;
    Ok(Some((
        BezierParameterRange2::new_validated(first_start, first_end),
        BezierParameterRange2::new_validated(second_start, second_end),
    )))
}

fn mapped_overlap_parameter(
    correspondence: &RationalBezierOverlapParameterCorrespondence2,
    first_to_second: bool,
    parameter: &BezierParameter2,
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<BezierParameter2> {
    let mapped = if first_to_second {
        correspondence.map_first_to_second(parameter, first_range, second_range, policy)
    } else {
        correspondence.map_second_to_first(parameter, first_range, second_range, policy)
    };
    match mapped {
        Ok(Classification::Decided(Some(parameter))) => Ok(parameter),
        Ok(Classification::Decided(None)) => Err(ExactCurveError::blocked(
            CurveOperation2::Boolean,
            family,
            UncertaintyReason::Predicate,
        )),
        Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
            CurveOperation2::Boolean,
            family,
            reason,
        )),
        Err(cause) => Err(ExactCurveError::invalid(
            CurveOperation2::Boolean,
            family,
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

    let (overlap_start, overlap_end) = ascending_range(first_range, policy)?;
    let (second_start_in_first, second_end_in_first) = if reversed {
        (
            second_carrier.end.unit_complement(),
            second_carrier.start.unit_complement(),
        )
    } else {
        (second_carrier.start.clone(), second_carrier.end.clone())
    };
    let start = maximum_parameter(
        [overlap_start, &first_carrier.start, &second_start_in_first],
        policy,
    )?;
    let end = minimum_parameter(
        [overlap_end, &first_carrier.end, &second_end_in_first],
        policy,
    )?;
    match decided_parameter_cmp(&start, &end, policy)? {
        Ordering::Less => {}
        Ordering::Equal | Ordering::Greater => {
            return Ok(CarrierOverlapClip::Matched(None));
        }
    }
    let second_start = map_to_second(&start);
    let second_end = map_to_second(&end);
    Ok(CarrierOverlapClip::Matched(Some((
        BezierParameterRange2::new_validated(start, end),
        BezierParameterRange2::new_validated(second_start, second_end),
    ))))
}

fn maximum_parameter<const N: usize>(
    parameters: [&BezierParameter2; N],
    policy: &CurveContext,
) -> ExactCurveResult<BezierParameter2> {
    let mut maximum = parameters[0];
    for parameter in &parameters[1..] {
        if decided_parameter_cmp(parameter, maximum, policy)?.is_gt() {
            maximum = parameter;
        }
    }
    Ok(maximum.clone())
}

fn minimum_parameter<const N: usize>(
    parameters: [&BezierParameter2; N],
    policy: &CurveContext,
) -> ExactCurveResult<BezierParameter2> {
    let mut minimum = parameters[0];
    for parameter in &parameters[1..] {
        if decided_parameter_cmp(parameter, minimum, policy)?.is_lt() {
            minimum = parameter;
        }
    }
    Ok(minimum.clone())
}

fn range_contains_fragment(
    range: &BezierParameterRange2,
    fragment_start: &BezierParameter2,
    fragment_end: &BezierParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (range_start, range_end) = ascending_range(range, policy)?;
    Ok(
        !decided_parameter_cmp(fragment_start, range_start, policy)?.is_lt()
            && !decided_parameter_cmp(fragment_end, range_end, policy)?.is_gt(),
    )
}

fn ascending_range<'a>(
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

fn decided_parameter_cmp(
    first: &BezierParameter2,
    second: &BezierParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<Ordering> {
    match first.cmp_by_refinement(second, policy).map_err(|cause| {
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

fn fragment_range(fragment: &BezierSplitFragment2) -> (&BezierParameter2, &BezierParameter2) {
    match fragment {
        BezierSplitFragment2::Materialized { start, end, .. }
        | BezierSplitFragment2::AlgebraicEndpointImages { start, end, .. }
        | BezierSplitFragment2::Unresolved { start, end } => (start, end),
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            (fragment.range().start(), fragment.range().end())
        }
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
            Self::AnalyticParallel(_) => CurveFamily2::RationalBezier,
        }
    }

    fn bezier(&self) -> &BezierSubcurve2 {
        match self {
            Self::Bezier(curve) => curve,
            Self::AnalyticParallel(_) => {
                unreachable!("parallel/rational dispatch requires a Bezier carrier")
            }
        }
    }

    fn parallel(&self) -> &BezierParallel2 {
        match self {
            Self::AnalyticParallel(parallel) => parallel,
            Self::Bezier(_) => {
                unreachable!("analytic pair dispatch requires a parallel carrier")
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
        }
    }

    fn certified_outer_bounds(&self, policy: &CurveContext) -> Classification<Aabb2> {
        match self {
            Self::Bezier(curve) => subcurve_certified_outer_bounds(curve, policy),
            Self::AnalyticParallel(parallel) => match parallel.conservative_bounds(policy) {
                Ok(bounds) => bounds,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
        }
    }

    fn has_certified_injective_axis(&self, policy: &CurveContext) -> bool {
        match self {
            Self::Bezier(curve) => subcurve_has_certified_injective_axis(curve, policy),
            Self::AnalyticParallel(parallel) => matches!(
                parallel.exact_rational_parallel_component(policy),
                Ok(Classification::Decided(Some(curve)))
                    if curve.has_certified_injective_axis(policy)
            ),
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
        }
    }
}

fn same_contact_point(
    first: &RationalBezierIntersectionPointEvidence2,
    second: &RationalBezierIntersectionPointEvidence2,
    policy: &CurveContext,
) -> Classification<bool> {
    match (first, second) {
        (
            RationalBezierIntersectionPointEvidence2::Exact(first),
            RationalBezierIntersectionPointEvidence2::Exact(second),
        ) => {
            if first.shares_storage(second) {
                return Classification::Decided(true);
            }
            match crate::classify::is_zero(&first.distance_squared(second), policy) {
                Some(equal) => Classification::Decided(equal),
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        (
            RationalBezierIntersectionPointEvidence2::Algebraic(first),
            RationalBezierIntersectionPointEvidence2::Algebraic(second),
        ) => {
            if let Some(classification) =
                first.same_injective_parametric_source_point(second, policy)
            {
                return classification;
            }
            // A decided same-sign rational Bezier control hull contains the
            // entire affine curve image, so disjoint source hulls prove that
            // these retained point images cannot represent the same contact.
            if let (
                Some(Classification::Decided(first_bounds)),
                Some(Classification::Decided(second_bounds)),
            ) = (
                first.parametric_source_bounds(policy),
                second.parametric_source_bounds(policy),
            ) && first_bounds.overlaps(&second_bounds, policy) == Classification::Decided(false)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "contact-point-equality",
                    "source-bounds-disjoint",
                );
                return Classification::Decided(false);
            }
            let (Some(first), Some(second)) = (first.resolved(policy), second.resolved(policy))
            else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let (Some(first_x), Some(first_y), Some(second_x), Some(second_y)) = (
                first.x().and_then(|image| image.representation()),
                first.y().and_then(|image| image.representation()),
                second.x().and_then(|image| image.representation()),
                second.y().and_then(|image| image.representation()),
            ) else {
                return if first == second {
                    Classification::Decided(true)
                } else {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                };
            };
            match (
                crate::bezier_arrangement::represented_roots_equal(first_x, second_x, policy),
                crate::bezier_arrangement::represented_roots_equal(first_y, second_y, policy),
            ) {
                (Some(x_equal), Some(y_equal)) => Classification::Decided(x_equal && y_equal),
                _ => Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        (
            RationalBezierIntersectionPointEvidence2::Exact(exact),
            RationalBezierIntersectionPointEvidence2::Algebraic(algebraic),
        )
        | (
            RationalBezierIntersectionPointEvidence2::Algebraic(algebraic),
            RationalBezierIntersectionPointEvidence2::Exact(exact),
        ) => {
            let Some(algebraic) = algebraic.resolved(policy) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let (Some(x), Some(y)) = (
                algebraic.x().and_then(|image| image.representation()),
                algebraic.y().and_then(|image| image.representation()),
            ) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let exact_x =
                crate::bezier_algebraic_image::exact_real_algebraic_representation(exact.x());
            let exact_y =
                crate::bezier_algebraic_image::exact_real_algebraic_representation(exact.y());
            match (
                crate::bezier_arrangement::represented_roots_equal(x, &exact_x, policy),
                crate::bezier_arrangement::represented_roots_equal(y, &exact_y, policy),
            ) {
                (Some(x_equal), Some(y_equal)) => Classification::Decided(x_equal && y_equal),
                _ => Classification::Uncertain(UncertaintyReason::RealSign),
            }
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
    use crate::{
        BezierAlgebraicParameter2, CurvePath2, LineSeg2, Point2, RationalBezier2,
        RationalBezierAlgebraicPointImage2, Real,
    };

    fn decided<T>(classification: Classification<T>) -> T {
        match classification {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                panic!("classification unexpectedly uncertain: {reason:?}")
            }
        }
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
            same_contact_point(&first, &second, &policy),
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
            same_contact_point(&first, &second, &policy),
            Classification::Decided(true)
        );
        #[cfg(feature = "predicates")]
        assert!(
            parameter
                .cached_rational_bezier_point_image(&curve)
                .is_none()
        );
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
