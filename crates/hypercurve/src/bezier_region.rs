//! Exact curved regions bounded by native and algebraic curve fragments.
//!
//! [`CurveRegion2`] is the top-level higher-order region type. It accepts
//! closed [`CurvePath2`] boundaries directly and materializes decided Boolean
//! traversals without flattening their native or algebraic carriers. It
//! deliberately does not force curved boundaries into line strings or expose
//! its private line/arc specialization, because the exactness model requires
//! exact curve objects to remain visible until a certified adapter exists.
//!
//! Exact area is exposed for polynomial Bezier loops and rational quadratic
//! conic loops whose homogeneous denominator is certified away from projective
//! zero on `[0, 1]`. Both use Green's-theorem boundary integrals, the same
//! identities used by [`crate::BezierAreaMoments2`]. Unsupported conic
//! denominator cases still return `None`
//! rather than silently sampling.

#[path = "curve_corner_chain.rs"]
pub(crate) mod curve_corner_chain;
use curve_corner_chain::CurveCornerChain2;

mod corner_edit;
mod offset_joins;
mod offset_spans;
use corner_edit::*;
use offset_joins::*;
pub(crate) use offset_spans::*;

use crate::CurvePointData2;
use crate::classify::product_sign;
use std::sync::Arc;
use std::sync::OnceLock;

use hyperreal::{Real, RealSign};
use hypersolve::{
    AlgebraicFiberRootCountStatus, BivariatePolynomial, CurveResultantParameter,
    count_bivariate_common_fiber_roots_at_algebraic_parameter,
};

use crate::bezier::BezierParallelLineTangentContact2;
use crate::bezier_algebraic_image::RationalBezierAlgebraicPointPredicate2;
use crate::bezier_moment::RationalQuadraticAreaIntegralCache;
use crate::bezier_offset::BezierAlgebraicCuspSemicircleSimilarityCache2;
use crate::bezier_offset::{
    BezierAlgebraicChordAxisDirection2, BezierAlgebraicFiberProjection2,
    algebraic_chord_point_linear_order_to_exact, algebraic_selected_correlated_predicate_sign,
    bivariate_fiber_strict_sign_on_parameter_range, selected_fiber_parameters,
};
use crate::bezier_split::BezierSelectedFiberSource2;
use crate::bezier_topology::exact_polynomial_line_contact_relation_from_direction;
use crate::classify::LineSide;
use crate::classify::{compare_reals, is_zero, real_sign};
use crate::curve::RetainedFilletRadialFrame2;
use crate::curve::{
    CornerPlacement2, CornerTrimCut2, RetainedFilletFrame2, compact_optional_corner_solutions,
    exact_corner_carrier, solve_exact_chamfer_corner, solve_exact_fillet_corner,
    try_map_corner_solutions, validate_corner_design_value,
};
use crate::curve_support::CurveSupport2;
use crate::policy::{
    PolicyClassificationCache, PolicyEvaluationCache, resolve_cached_classification,
    resolve_cached_evaluation, resolve_certified_operation, resolve_certified_value,
};
use crate::region::LineArcRegion2;
use crate::region_nesting::assemble_unordered_segment_rings;
use crate::{
    Aabb2, BezierAlgebraicEndpointImage2, BezierAreaMoments2, BezierArrangementGraph2,
    BezierArrangementTraversal2, BezierEndpoint, BezierFlatteningOptions, BezierLineContact,
    BezierLineContactKind, BezierLineContactRelation, BezierLineCrossingDirection,
    BezierLineImageFitRelation, BezierParallel2, BezierParallelSource2, BezierParameter2,
    BezierParameterRange2, BezierSplitFragment2, BezierSubcurve2, BooleanOp, CircularArc2,
    Classification, Contour2, ContourPointLocation, CubicBezier2, Curve2, CurveCertainty,
    CurveContext, CurveCornerMode2, CurveCornerSolutions2, CurveError, CurveFamily2, CurveFillet2,
    CurveGeometry2, CurveIntersectionPairBlockerKind2, CurveOperation2, CurveOutcome,
    CurveParameter2, CurveParameterRange2, CurveParameterSide2, CurvePath2,
    CurvePathIntersectionContact2, CurvePoint2, CurveResult, ExactCurveError, ExactCurveResult,
    FillRule, LineSeg2, OffsetCap, OffsetCornerStyle2, Point2, QuadraticBezier2, RationalBezier2,
    RationalBezierAlgebraicPointImage2, RationalBezierPointIncidence2, RationalQuadraticBezier2,
    RegionPointLocation, Segment2, UncertaintyReason,
};

/// A closed native Bezier/conic boundary loop.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierBoundaryLoop2 {
    fragments: Vec<BezierSubcurve2>,
}

/// A closed boundary of exact curves, with retained endpoint connectivity.
///
/// [`Self::curves`] exposes authored and generated curves through the same
/// interface while preserving their selected parameters and construction evidence.
#[derive(Clone, Debug)]
pub struct CurveRegionBoundaryLoop2 {
    fragments: Vec<BezierSplitFragment2>,
    connectivity_policy: Option<CurveContext>,
    curves: OnceLock<Arc<[Curve2]>>,
    rational_evaluators: OnceLock<CurveResult<Vec<Option<RationalBezier2>>>>,
    /// Certified query bounds of every fragment under the first policy that
    /// asked. Point queries otherwise rebuild each box per query.
    query_bounds: OnceLock<(CurveContext, Arc<[FragmentQueryBounds]>, Option<[f64; 4]>)>,
    arrangement_sources: Option<Vec<CurveRegionFragmentSource2>>,
}

impl CurveRegionBoundaryLoop2 {
    /// Certified query bounds for every fragment, shared by all point
    /// queries under the cached policy. Other policies compute their own.
    fn fragment_query_bounds(
        &self,
        policy: &CurveContext,
    ) -> Option<(&[FragmentQueryBounds], Option<[f64; 4]>)> {
        let (cached_policy, bounds, loop_box) = self.query_bounds.get_or_init(|| {
            let bounds = self
                .fragments
                .iter()
                .map(|fragment| {
                    let exact = retained_fragment_query_bounds(fragment, policy);
                    let approximate = match &exact {
                        Classification::Decided(bounds) => f64_box(bounds),
                        Classification::Uncertain(_) => None,
                    };
                    FragmentQueryBounds { exact, approximate }
                })
                .collect::<Arc<[_]>>();
            // Certified union of every fragment box, when each has one.
            let loop_box = bounds.iter().try_fold(
                [
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                ],
                |union, fragment| {
                    let bounds = fragment.approximate?;
                    Some([
                        union[0].min(bounds[0]),
                        union[1].max(bounds[1]),
                        union[2].min(bounds[2]),
                        union[3].max(bounds[3]),
                    ])
                },
            );
            (
                *policy,
                bounds,
                loop_box.filter(|_| !self.fragments.is_empty()),
            )
        });
        (cached_policy == policy).then_some((bounds, *loop_box))
    }
}

/// Certified fragment bounds plus an f64 image used only for conservative
/// rejection before the exact predicate.
#[derive(Clone, Debug)]
struct FragmentQueryBounds {
    exact: Classification<Aabb2>,
    approximate: Option<[f64; 4]>,
}

/// Certified outward f64 bounds of a `Real`: exact rationals directly, other
/// values through a certified rational interval. Aborted evaluation or
/// out-of-range values have none and stay on the exact path.
pub(crate) fn certified_f64_enclosure(value: &Real) -> Option<[f64; 2]> {
    if let Some(exact) = value.exact_rational_ref() {
        return exact.to_f64_enclosure();
    }
    let [lower, upper] = value.certified_rational_interval(-64)?;
    Some([lower.to_f64_enclosure()?[0], upper.to_f64_enclosure()?[1]])
}

fn f64_box(bounds: &Aabb2) -> Option<[f64; 4]> {
    Some([
        certified_f64_enclosure(bounds.min().x())?[0],
        certified_f64_enclosure(bounds.max().x())?[1],
        certified_f64_enclosure(bounds.min().y())?[0],
        certified_f64_enclosure(bounds.max().y())?[1],
    ])
}

/// Returns true only when an f64 evaluation proves the box misses the
/// forward ray: every corner lies strictly on one side of the supporting
/// line or strictly behind the origin, which is the exact test's pruning
/// condition. Inputs are certified enclosures (within one ulp) and each
/// projection takes a few roundings, so its forward error is a small multiple
/// of the unit roundoff times `scale`; the margin exceeds that by orders of
/// magnitude. Inconclusive results fall back to the exact test.
fn f64_box_certainly_misses_forward_ray(
    bounds: [f64; 4],
    origin: [[f64; 2]; 2],
    direction: [[f64; 2]; 2],
) -> bool {
    const MARGIN: f64 = 1.0e-9;
    // Midpoints and radii of the certified origin and direction intervals.
    let mid_radius = |[low, high]: [f64; 2]| ((low + high) * 0.5, (high - low) * 0.5);
    let (origin_x, origin_x_radius) = mid_radius(origin[0]);
    let (origin_y, origin_y_radius) = mid_radius(origin[1]);
    let (dx, dx_radius) = mid_radius(direction[0]);
    let (dy, dy_radius) = mid_radius(direction[1]);
    // |ab - a'b'| <= |a'| r_b + |b'| r_a + r_a r_b for a in a' +- r_a.
    let product_error = |a: f64, a_radius: f64, b: f64, b_radius: f64| {
        a.abs() * b_radius + b.abs() * a_radius + a_radius * b_radius
    };
    let mut side_signs = [true, true];
    let mut all_behind = true;
    for (x, y) in [
        (bounds[0], bounds[2]),
        (bounds[0], bounds[3]),
        (bounds[1], bounds[2]),
        (bounds[1], bounds[3]),
    ] {
        let delta_x = x - origin_x;
        let delta_y = y - origin_y;
        let scale = (x.abs() + origin_x.abs()) * (dx.abs() + dy.abs())
            + (y.abs() + origin_y.abs()) * (dx.abs() + dy.abs());
        let input_error = product_error(dy, dy_radius, delta_x, origin_x_radius)
            + product_error(dx, dx_radius, delta_y, origin_y_radius)
            + product_error(dx, dx_radius, delta_x, origin_x_radius)
            + product_error(dy, dy_radius, delta_y, origin_y_radius);
        let error = MARGIN * (scale + input_error) + input_error + 1.0e-300;
        let side = -dy * delta_x + dx * delta_y;
        let forward = dx * delta_x + dy * delta_y;
        if side >= -error {
            side_signs[0] = false;
        }
        if side <= error {
            side_signs[1] = false;
        }
        if forward >= -error {
            all_behind = false;
        }
    }
    side_signs[0] || side_signs[1] || all_behind
}

impl PartialEq for CurveRegionBoundaryLoop2 {
    fn eq(&self, other: &Self) -> bool {
        let fragment_count = self.fragments.len();
        fragment_count == other.fragments.len()
            && (fragment_count == 0
                || other
                    .fragments
                    .iter()
                    .enumerate()
                    .filter(|(_, fragment)| *fragment == &self.fragments[0])
                    .any(|(offset, _)| {
                        self.fragments
                            .iter()
                            .zip(other.fragments.iter().cycle().skip(offset))
                            .all(|(left, right)| left == right)
                    }))
    }
}

/// Provenance recorded while constructing a retained boundary fragment.
///
/// Curve operations supply these records; completed boundaries expose them
/// for inspection without accepting caller-authored arrangement indices.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct CurveRegionFragmentSource2 {
    arrangement_fragment_index: usize,
    source_curve_index: usize,
    source_fragment_index: usize,
}

impl CurveRegionFragmentSource2 {
    /// Constructs retained fragment provenance from arrangement graph indices.
    pub(crate) const fn new(
        arrangement_fragment_index: usize,
        source_curve_index: usize,
        source_fragment_index: usize,
    ) -> Self {
        Self {
            arrangement_fragment_index,
            source_curve_index,
            source_fragment_index,
        }
    }

    /// Returns the retained arrangement-graph fragment index.
    pub const fn arrangement_fragment_index(self) -> usize {
        self.arrangement_fragment_index
    }
}

/// An exact regularized planar filled set with mixed-family boundary curves.
///
/// Public constructors resolve authored winding and canceled seams before
/// publishing the region. Its boundary retains selected-root and endpoint
/// evidence without requiring stored Cartesian coordinates. Exact area
/// integrals are optional; membership and region operations use certified
/// boundary topology.
#[derive(Clone)]
pub struct CurveRegion2 {
    data: Arc<CurveRegionData2>,
}

struct CurveRegionData2 {
    boundary_loops: Vec<CurveRegionBoundaryLoop2>,
    certified_loop_roles: Option<Arc<[CurveRegionLoopRole]>>,
    state: CurveRegionState2,
    certified_regularization: OnceLock<CurveRegion2>,
    strict_materialized_connectivity_certified: bool,
    filled_side_is_left: PolicyClassificationCache<Arc<[bool]>>,
    native_boundary_loops: OnceLock<Option<Arc<[BezierBoundaryLoop2]>>>,
    native_boundary_bounds: PolicyClassificationCache<Arc<[Aabb2]>>,
    line_image_region: PolicyClassificationCache<Option<LineArcRegion2>>,
    signed_area_cache: PolicyEvaluationCache<Option<Real>>,
}

/// The semantic state of a region's boundary loops.
///
/// A normalized region has filled-left loops with winding 0 or +/-1, so no
/// per-loop fill rule can change its set; authored fill rules exist only on
/// a raw loop set still being evaluated. One state makes the two exclusive.
#[derive(Clone, Debug, PartialEq)]
enum CurveRegionState2 {
    /// Retained loops with no authored winding semantics.
    Raw,
    /// Authored loops whose per-loop fill rules define the set until
    /// regularization evaluates them.
    Authored(Arc<[FillRule]>),
    /// A certified regularized, filled-left boundary under this policy.
    Normalized(CurveContext),
}

impl CurveRegionData2 {
    fn authored_fill_rules(&self) -> Option<&[FillRule]> {
        match &self.state {
            CurveRegionState2::Authored(rules) => Some(rules),
            CurveRegionState2::Raw | CurveRegionState2::Normalized(_) => None,
        }
    }

    const fn normalized_policy(&self) -> Option<CurveContext> {
        match self.state {
            CurveRegionState2::Normalized(policy) => Some(policy),
            CurveRegionState2::Raw | CurveRegionState2::Authored(_) => None,
        }
    }

    fn new(boundary_loops: Vec<CurveRegionBoundaryLoop2>) -> Self {
        let strict_materialized_connectivity_certified =
            retained_region_has_strict_materialized_connectivity(&boundary_loops);
        Self {
            boundary_loops,
            certified_loop_roles: None,
            state: CurveRegionState2::Raw,
            certified_regularization: OnceLock::new(),
            strict_materialized_connectivity_certified,
            filled_side_is_left: PolicyClassificationCache::new(),
            native_boundary_loops: OnceLock::new(),
            native_boundary_bounds: PolicyClassificationCache::new(),
            line_image_region: PolicyClassificationCache::new(),
            signed_area_cache: PolicyEvaluationCache::new(),
        }
    }
}

fn retained_region_has_strict_materialized_connectivity(
    boundary_loops: &[CurveRegionBoundaryLoop2],
) -> bool {
    boundary_loops.iter().all(|boundary_loop| {
        let fragments = boundary_loop.fragments();
        fragments
            .iter()
            .zip(fragments.iter().cycle().skip(1))
            .take(fragments.len())
            .all(|(left, right)| {
                let (
                    BezierSplitFragment2::Materialized {
                        curve: left_curve, ..
                    },
                    BezierSplitFragment2::Materialized {
                        curve: right_curve, ..
                    },
                ) = (left, right)
                else {
                    return false;
                };
                left_curve.endpoint_refs().1 == right_curve.endpoint_refs().0
            })
    })
}

fn shared_empty_curve_region_data() -> Arc<CurveRegionData2> {
    static EMPTY: OnceLock<Arc<CurveRegionData2>> = OnceLock::new();
    Arc::clone(EMPTY.get_or_init(|| {
        let mut data = CurveRegionData2::new(Vec::new());
        data.state = CurveRegionState2::Normalized(CurveContext::STRICT);
        data.certified_loop_roles = Some(Arc::from(Vec::new()));
        data.filled_side_is_left.certify(Arc::from(Vec::new()));
        data.line_image_region
            .certify(Some(LineArcRegion2::empty()));
        Arc::new(data)
    }))
}

impl Default for CurveRegion2 {
    fn default() -> Self {
        Self {
            data: shared_empty_curve_region_data(),
        }
    }
}

/// Borrowed exact line/arc output adapter for a [`CurveRegion2`].
///
/// This exposes zero-copy specialized geometry without transferring ownership
/// of the kernel's private native-region carrier or creating another operation
/// authority. Higher-order regions return explicit `Unsupported` uncertainty.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurveRegionNativeContourView2<'a> {
    material_contours: &'a [Contour2],
    hole_contours: &'a [Contour2],
}

impl<'a> CurveRegionNativeContourView2<'a> {
    /// Returns material contours in native fast-path order.
    pub const fn material_contours(&self) -> &'a [Contour2] {
        self.material_contours
    }

    /// Returns hole contours in native fast-path order.
    pub const fn hole_contours(&self) -> &'a [Contour2] {
        self.hole_contours
    }

    /// Returns true when both native contour bins are empty.
    pub const fn is_empty(&self) -> bool {
        self.material_contours.is_empty() && self.hole_contours.is_empty()
    }

    /// Returns total native boundary contour count.
    pub const fn len(&self) -> usize {
        self.material_contours.len() + self.hole_contours.len()
    }
}

/// Certified source-segmentation evidence for one region boundary loop used by an offset.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionSegmentationLoopEvidence2 {
    role: CurveRegionLoopRole,
    fill_rule: FillRule,
    source_curve_count: usize,
    source_fragment_count: usize,
    output_segment_count: usize,
    max_depth: usize,
}

/// Exact-scalar chordization evidence for every unified region boundary loop.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionCertifiedSegmentationEvidence2 {
    max_source_chord_error: Real,
    loop_evidence: Vec<CurveRegionSegmentationLoopEvidence2>,
    lossy_boundary: bool,
}

/// A line-only unified region emitted by certified exact-scalar segmentation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionCertifiedSegmentationResult2 {
    region: CurveRegion2,
    evidence: CurveRegionCertifiedSegmentationEvidence2,
}

impl CurveRegionSegmentationLoopEvidence2 {
    /// Returns the authoritative role retained for this source loop.
    pub const fn role(&self) -> CurveRegionLoopRole {
        self.role
    }

    /// Returns the source loop's fill rule.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns authored top-level curve count in this loop.
    pub const fn source_curve_count(&self) -> usize {
        self.source_curve_count
    }

    /// Returns native Bezier/conic span count covered by segmentation.
    pub const fn source_fragment_count(&self) -> usize {
        self.source_fragment_count
    }

    /// Returns exact line-segment count emitted for the approximating loop.
    pub const fn output_segment_count(&self) -> usize {
        self.output_segment_count
    }

    /// Returns maximum subdivision depth used by any source span.
    pub const fn max_depth(&self) -> usize {
        self.max_depth
    }
}

impl CurveRegionCertifiedSegmentationEvidence2 {
    /// Returns the certified source-curve-to-chord error budget.
    pub const fn max_source_chord_error(&self) -> &Real {
        &self.max_source_chord_error
    }

    /// Returns one exact-scalar segmentation record per retained loop.
    pub fn loop_evidence(&self) -> &[CurveRegionSegmentationLoopEvidence2] {
        &self.loop_evidence
    }

    /// Returns true because replacing a non-line curve by chords is a lossy boundary.
    pub const fn lossy_boundary(&self) -> bool {
        self.lossy_boundary
    }
}

impl CurveRegionCertifiedSegmentationResult2 {
    /// Returns the line-only unified region produced by chordization.
    pub const fn region(&self) -> &CurveRegion2 {
        &self.region
    }

    /// Returns retained role, fill, and error-budget evidence.
    pub const fn evidence(&self) -> &CurveRegionCertifiedSegmentationEvidence2 {
        &self.evidence
    }

    /// Consumes the result and returns its line-only unified region.
    pub fn into_region(self) -> CurveRegion2 {
        self.region
    }

    /// Consumes the result into its line-only region and evidence.
    pub fn into_parts(self) -> (CurveRegion2, CurveRegionCertifiedSegmentationEvidence2) {
        (self.region, self.evidence)
    }
}

impl std::fmt::Debug for CurveRegion2 {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("CurveRegion2")
            .field("boundary_loops", &self.data.boundary_loops)
            .field("certified_loop_roles", &self.data.certified_loop_roles)
            .field("state", &self.data.state)
            .finish()
    }
}

impl PartialEq for CurveRegion2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.boundary_loops == other.data.boundary_loops
                && self.data.certified_loop_roles == other.data.certified_loop_roles
                && self.data.state == other.data.state)
    }
}

/// Filled side of an oriented closed curve boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CurveBoundaryInteriorSide2 {
    /// Material lies to the left while traversing the boundary.
    Left,
    /// Material lies to the right while traversing the boundary.
    Right,
}

/// Material/hole role assigned to one retained Bezier boundary loop.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveRegionLoopRole {
    /// The loop contributes filled material.
    Material,
    /// The loop subtracts from the containing material loop.
    Hole,
}

fn shared_curve_region_loop_roles(roles: Vec<CurveRegionLoopRole>) -> Arc<[CurveRegionLoopRole]> {
    static MATERIAL_HOLE: OnceLock<Arc<[CurveRegionLoopRole]>> = OnceLock::new();
    match roles.as_slice() {
        [CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole] => MATERIAL_HOLE
            .get_or_init(|| Arc::from([CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]))
            .clone(),
        _ => roles.into(),
    }
}

fn shared_all_material_curve_region_loop_roles(role_count: usize) -> Arc<[CurveRegionLoopRole]> {
    static ONE_MATERIAL: OnceLock<Arc<[CurveRegionLoopRole]>> = OnceLock::new();
    static TWO_MATERIAL: OnceLock<Arc<[CurveRegionLoopRole]>> = OnceLock::new();
    match role_count {
        1 => ONE_MATERIAL
            .get_or_init(|| Arc::from([CurveRegionLoopRole::Material]))
            .clone(),
        2 => TWO_MATERIAL
            .get_or_init(|| {
                Arc::from([CurveRegionLoopRole::Material, CurveRegionLoopRole::Material])
            })
            .clone(),
        _ => std::iter::repeat_n(CurveRegionLoopRole::Material, role_count)
            .collect::<Vec<_>>()
            .into(),
    }
}

/// One exact retained material boundary and the hole boundaries it owns.
///
/// Ownership is classified against retained curve carriers before finite
/// projection, so meshing adapters never need to infer topology from samples.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionProfile2<'a> {
    material_loop_index: usize,
    material: &'a CurveRegionBoundaryLoop2,
    hole_loop_indices: Vec<usize>,
    holes: Vec<&'a CurveRegionBoundaryLoop2>,
}

impl<'a> CurveRegionProfile2<'a> {
    /// Returns the material loop's index in its source region.
    pub const fn material_loop_index(&self) -> usize {
        self.material_loop_index
    }

    /// Returns the retained material boundary.
    pub const fn material(&self) -> &'a CurveRegionBoundaryLoop2 {
        self.material
    }

    /// Returns source-region indices for the owned holes.
    pub fn hole_loop_indices(&self) -> &[usize] {
        &self.hole_loop_indices
    }

    /// Returns the retained hole boundaries owned by this material boundary.
    pub fn holes(&self) -> &[&'a CurveRegionBoundaryLoop2] {
        &self.holes
    }
}

/// Internal result of nesting raw native loops. Boundary provenance remains
/// owned by the region; the caller consumes only roles and signed areas.
struct NativeLoopNesting2 {
    roles: Vec<CurveRegionLoopRole>,
    signed_areas: Vec<Real>,
}

impl BezierBoundaryLoop2 {
    /// Returns native curve fragments in loop order.
    pub fn fragments(&self) -> &[BezierSubcurve2] {
        &self.fragments
    }

    /// Consumes the loop and returns native curve fragments.
    pub fn into_fragments(self) -> Vec<BezierSubcurve2> {
        self.fragments
    }

    pub(crate) fn signed_area_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        let mut rational_quadratic_cache = RationalQuadraticAreaIntegralCache::default();
        self.signed_area_with_cache(policy, &mut rational_quadratic_cache)
    }

    fn signed_area_with_cache(
        &self,
        policy: &CurveContext,
        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Classification<Option<Real>>> {
        if self.fragments.is_empty() {
            return Err(CurveError::Topology(
                "Bezier boundary loop signed area requires nonempty fragments".to_owned(),
            ));
        }

        let mut total = Real::zero();
        for fragment in &self.fragments {
            match fragment.signed_area_contribution_with_cache(policy, rational_quadratic_cache)? {
                Classification::Decided(Some(contribution)) => {
                    total = &total + &contribution;
                }
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        Ok(Classification::Decided(Some(total)))
    }
}

fn boundary_area_moments<'a>(
    fragments: impl Iterator<Item = Option<&'a BezierSubcurve2>>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierAreaMoments2>>> {
    let mut total = BezierAreaMoments2::zero();
    for fragment in fragments {
        let Some(fragment) = fragment else {
            return Ok(Classification::Decided(None));
        };
        match fragment.area_moments_contribution_raw(policy)? {
            Classification::Decided(Some(contribution)) => {
                total = total.plus(&contribution);
            }
            Classification::Decided(None) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    }
    Ok(Classification::Decided(Some(total)))
}

impl From<BezierBoundaryLoop2> for CurveRegionBoundaryLoop2 {
    fn from(boundary_loop: BezierBoundaryLoop2) -> Self {
        Self {
            fragments: boundary_loop
                .into_fragments()
                .into_iter()
                .map(|curve| BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve,
                })
                .collect(),
            arrangement_sources: None,
            connectivity_policy: None,
            curves: OnceLock::new(),
            rational_evaluators: OnceLock::new(),
            query_bounds: OnceLock::new(),
        }
    }
}

impl BezierSubcurve2 {
    pub(crate) fn endpoint_refs(&self) -> (&Point2, &Point2) {
        match self {
            Self::Quadratic(curve) => (curve.start(), curve.end()),
            Self::Cubic(curve) => (curve.start(), curve.end()),
            Self::RationalQuadratic(curve) => (curve.start(), curve.end()),
            Self::Rational(curve) => (curve.start(), curve.end()),
        }
    }

    /// Collapses an exact rational circle to its quadratic parameter frame and
    /// attaches the one normalized circle certificate used by every kernel.
    pub(crate) fn canonical_rational_circle_quadratic(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalQuadraticBezier2>>> {
        let quadratic = match self {
            Self::RationalQuadratic(curve) => curve.clone(),
            Self::Rational(curve) => match curve.materialized_quadratic_representative(policy)? {
                Classification::Decided(Some(curve)) => curve,
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
            Self::Quadratic(_) | Self::Cubic(_) => {
                return Ok(Classification::Decided(None));
            }
        };
        let retained_implicit = quadratic.retained_implicit_quadratic_conic().cloned();
        let retained_circular = quadratic.retained_circular_conic().cloned();
        if retained_implicit.is_some() && retained_circular.is_some() {
            return Ok(Classification::Decided(Some(quadratic)));
        }
        let support = match crate::arc_bezier::rational_quadratic_circular_arc(&quadratic, policy)?
        {
            Classification::Decided(Some(support)) => support,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (implicit, circular) = crate::arc_bezier::circular_conic_provenance(&support);
        Ok(Classification::Decided(Some(
            quadratic.with_retained_conic_provenance(
                Some(retained_implicit.unwrap_or(implicit)),
                Some(retained_circular.unwrap_or(circular)),
            ),
        )))
    }
}

fn canonicalize_exact_rational_subcurve(
    curve: BezierSubcurve2,
    policy: &CurveContext,
) -> BezierSubcurve2 {
    // Canonicalization may consume only a STRICT proof. APPROXIMATE_512 is a
    // terminal predicate policy and must never become reusable construction
    // provenance. Exact inverse elevation preserves the authored local
    // parameter and lineage while preventing structural degree from inflating
    // every downstream resultant.
    let strict = policy.strict_counterpart();
    let (curve, structurally_reduced) = match curve {
        BezierSubcurve2::Rational(source) => {
            match source.retained_minimal_degree_representative(&strict) {
                Ok(Classification::Decided(Some(reduced))) => {
                    (BezierSubcurve2::Rational(reduced), true)
                }
                Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => {
                    (BezierSubcurve2::Rational(source), false)
                }
            }
        }
        curve => (curve, false),
    };
    match curve.canonical_rational_circle_quadratic(&strict) {
        Ok(Classification::Decided(Some(quadratic))) => {
            return BezierSubcurve2::RationalQuadratic(quadratic);
        }
        Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => {}
    }
    if structurally_reduced
        && let BezierSubcurve2::Rational(source) = &curve
        && source.degree() == 2
        && let Ok(Classification::Decided(Some(quadratic))) =
            source.materialized_quadratic_representative(&strict)
    {
        return BezierSubcurve2::RationalQuadratic(quadratic);
    }
    curve
}

fn canonicalize_retained_rational_fragment(
    fragment: BezierSplitFragment2,
    policy: &CurveContext,
) -> BezierSplitFragment2 {
    let BezierSplitFragment2::Materialized { start, end, curve } = fragment else {
        return fragment;
    };
    BezierSplitFragment2::Materialized {
        start,
        end,
        curve: canonicalize_exact_rational_subcurve(curve, policy),
    }
}

impl CurveRegionBoundaryLoop2 {
    pub(crate) fn from_path(
        path: &CurvePath2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Self>> {
        match crate::curve::validate_closed_curve_path_connectivity(path, policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let mut fragments = Vec::with_capacity(path.curves().len());
        for curve in path.curves() {
            if let Some(fragment) = curve.retained_fragment() {
                fragments.push(fragment.clone());
            } else if let Some(spans) =
                curve.restricted_source_spans(policy, CurveOperation2::Arrangement)?
            {
                for adjacent in spans.windows(2) {
                    match curve_fragment_endpoints_equal(
                        &adjacent[0].fragment,
                        false,
                        &adjacent[1].fragment,
                        true,
                        policy,
                    ) {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => {
                            return Err(ExactCurveError::invalid(
                                CurveOperation2::Arrangement,
                                curve.family(),
                                CurveError::DisconnectedCurvePath,
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                fragments.extend(spans.iter().map(|span| span.fragment.clone()));
            } else {
                let native = match curve.native_bezier_fragments_with_policy(policy)? {
                    Classification::Decided(native) => native,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                // Path joins certify the authored curves' outer endpoints.
                // A spline can still have a discontinuous interior knot, so
                // certify its promoted span joins before retaining a cycle.
                for adjacent in native.windows(2) {
                    let (_, left_end) = adjacent[0].native_curve().endpoint_refs();
                    let (right_start, _) = adjacent[1].native_curve().endpoint_refs();
                    match CurvePoint2::from(left_end.clone())
                        .same_point(&CurvePoint2::from(right_start.clone()), policy)
                    {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => {
                            return Err(ExactCurveError::invalid(
                                CurveOperation2::Arrangement,
                                curve.family(),
                                CurveError::DisconnectedCurvePath,
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                fragments.extend(
                    native
                        .iter()
                        .map(|native| BezierSplitFragment2::Materialized {
                            start: BezierParameter2::Exact(Real::zero()),
                            end: BezierParameter2::Exact(Real::one()),
                            curve: native.native_curve().clone(),
                        }),
                );
            }
        }
        let fragments = fragments
            .into_iter()
            .map(|fragment| canonicalize_retained_rational_fragment(fragment, policy))
            .collect();
        Self::try_new_from_certified_connected_chain(fragments, None, policy)
            .map(Classification::Decided)
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Arrangement,
                    path.curves()[0].family(),
                    cause,
                )
            })
    }

    fn rational_evaluators(&self) -> CurveResult<&[Option<RationalBezier2>]> {
        match self.rational_evaluators.get_or_init(|| {
            self.fragments
                .iter()
                .map(|fragment| match fragment {
                    BezierSplitFragment2::RetainedBezier { source_curve, .. } => {
                        rationalize_retained_subcurve(source_curve).map(Some)
                    }
                    _ => Ok(None),
                })
                .collect()
        }) {
            Ok(evaluators) => Ok(evaluators),
            Err(error) => Err(error.clone()),
        }
    }

    pub(crate) fn classify_point_raw(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<ContourPointLocation>> {
        classify_point_against_retained_loop(self, point, policy)
    }

    /// Returns exact area and first moments when every fragment has an
    /// implemented native symbolic integral; otherwise returns `Decided(None)`.
    pub fn area_moments(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Option<BezierAreaMoments2>>>> {
        resolve_certified_operation(policy, |attempt| {
            boundary_area_moments(
                self.fragments.iter().map(|fragment| match fragment {
                    BezierSplitFragment2::Materialized { curve, .. } => Some(curve),
                    _ => None,
                }),
                attempt,
            )
        })
    }

    /// Constructs a retained boundary loop from accepted split fragments.
    pub(crate) fn new(
        fragments: Vec<BezierSplitFragment2>,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        let fragments = fragments
            .into_iter()
            .map(|fragment| canonicalize_retained_rational_fragment(fragment, policy))
            .collect::<Vec<_>>();
        validate_retained_boundary_loop(&fragments, policy)?;
        Ok(Self {
            fragments,
            arrangement_sources: None,
            connectivity_policy: Some(policy.retained_object_policy()),
            curves: OnceLock::new(),
            rational_evaluators: OnceLock::new(),
            query_bounds: OnceLock::new(),
        })
    }

    fn try_new_from_certified_arrangement_chain(
        fragments: Vec<BezierSplitFragment2>,
        arrangement_sources: Vec<CurveRegionFragmentSource2>,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        if fragments.is_empty() || fragments.len() != arrangement_sources.len() {
            return Err(CurveError::Topology(
                "certified arrangement chain has inconsistent retained fragments".into(),
            ));
        }
        for fragment in &fragments {
            validate_retained_fragment_provenance(fragment, policy)?;
        }
        validate_retained_boundary_loop_sources(&arrangement_sources)?;
        Ok(Self::from_certified_arrangement_chain(
            fragments,
            arrangement_sources,
            policy,
        ))
    }

    fn from_certified_arrangement_chain(
        mut fragments: Vec<BezierSplitFragment2>,
        arrangement_sources: Vec<CurveRegionFragmentSource2>,
        policy: &CurveContext,
    ) -> Self {
        debug_assert!(!fragments.is_empty());
        debug_assert_eq!(fragments.len(), arrangement_sources.len());
        share_line_image_vertices(&mut fragments, policy);
        Self {
            fragments,
            arrangement_sources: Some(arrangement_sources),
            connectivity_policy: Some(policy.retained_object_policy()),
            curves: OnceLock::new(),
            rational_evaluators: OnceLock::new(),
            query_bounds: OnceLock::new(),
        }
    }

    /// Constructs a loop whose complete edge-to-edge connectivity was proved
    /// by one authoritative caller before fragment materialization.
    ///
    /// Exact offset joins use tangent/contact maps that can certify a shared
    /// vertex even when the two retained endpoint images inhabit independent
    /// selected fields. Re-running the generic Cartesian endpoint predicate
    /// here would discard that stronger pair-owned proof. Fragment provenance
    /// and optional source records remain validated locally; the caller must
    /// have decided every cyclic join before invoking this constructor.
    pub(crate) fn try_new_from_certified_connected_chain(
        mut fragments: Vec<BezierSplitFragment2>,
        arrangement_sources: Option<Vec<CurveRegionFragmentSource2>>,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        if fragments.is_empty() {
            return Err(CurveError::Topology(
                "certified retained boundary chain requires nonempty fragments".into(),
            ));
        }
        for fragment in &fragments {
            validate_retained_fragment_provenance(fragment, policy)?;
        }
        if let Some(sources) = &arrangement_sources {
            if fragments.len() != sources.len() {
                return Err(CurveError::Topology(
                    "certified retained boundary source count does not match fragment count".into(),
                ));
            }
            validate_retained_boundary_loop_sources(sources)?;
        }
        share_line_image_vertices(&mut fragments, policy);
        Ok(Self {
            fragments,
            arrangement_sources,
            connectivity_policy: Some(policy.retained_object_policy()),
            curves: OnceLock::new(),
            rational_evaluators: OnceLock::new(),
            query_bounds: OnceLock::new(),
        })
    }

    /// Returns each exact boundary curve in traversal order.
    ///
    /// Analytic parallels, algebraic chords, selected circles, and selected-fiber
    /// cuts are [`Curve2`] values. Later Boolean, offset, fillet, and chamfer
    /// steps accept those values directly. [`Curve2::geometry`] remains the
    /// optional native definition.
    ///
    /// Repeated access borrows the same curves and their cached calculations.
    pub fn curves(&self) -> &[Curve2] {
        self.curves.get_or_init(|| {
            self.fragments
                .iter()
                .cloned()
                .map(Curve2::from_retained_fragment)
                .collect()
        })
    }

    pub(crate) fn fragments(&self) -> &[BezierSplitFragment2] {
        &self.fragments
    }

    pub(crate) fn without_arrangement_sources(mut self) -> Self {
        self.arrangement_sources = None;
        self
    }

    /// Returns arrangement/source indices for graph-built loops, when retained.
    pub(crate) fn arrangement_sources(&self) -> Option<&[CurveRegionFragmentSource2]> {
        self.arrangement_sources.as_deref()
    }

    /// Returns true when every retained fragment has graph source provenance.
    pub const fn has_arrangement_sources(&self) -> bool {
        self.arrangement_sources.is_some()
    }

    /// Returns the number of retained fragments in the loop.
    pub fn len(&self) -> usize {
        self.fragments.len()
    }

    /// Returns true when the loop contains no fragments.
    pub fn is_empty(&self) -> bool {
        self.fragments.is_empty()
    }

    /// Returns true when any retained fragment carries non-native algebraic geometry.
    pub fn has_algebraic_fragments(&self) -> bool {
        self.fragments
            .iter()
            .any(|fragment| !matches!(fragment, BezierSplitFragment2::Materialized { .. }))
    }

    /// Returns exact signed area for implemented native integrals and certified
    /// exact line-image fragments.
    pub fn signed_area(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Option<Real>>>> {
        resolve_certified_operation(policy, |attempt| self.signed_area_raw(attempt))
    }

    pub(crate) fn signed_area_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        let mut rational_quadratic_cache = RationalQuadraticAreaIntegralCache::default();
        self.signed_area_with_cache(policy, &mut rational_quadratic_cache)
    }

    fn signed_area_with_cache(
        &self,
        policy: &CurveContext,
        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Classification<Option<Real>>> {
        if self.fragments.is_empty() {
            return Err(CurveError::Topology(
                "retained Bezier boundary loop signed area requires nonempty fragments".to_owned(),
            ));
        }

        let mut total = Real::zero();
        for fragment in &self.fragments {
            if let BezierSplitFragment2::Materialized { curve, .. } = fragment {
                match curve.signed_area_contribution_with_cache(policy, rational_quadratic_cache)? {
                    Classification::Decided(Some(contribution)) => {
                        total = &total + &contribution;
                    }
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                continue;
            }
            let line = match retained_line_fragment_segment(fragment, policy)? {
                Classification::Decided(line) => line,
                Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
            };
            total =
                &total + &crate::contour::line_signed_area_contribution(line.start(), line.end())?;
        }
        Ok(Classification::Decided(Some(total)))
    }
}

fn validate_retained_boundary_loop(
    fragments: &[BezierSplitFragment2],
    policy: &CurveContext,
) -> CurveResult<()> {
    if fragments.is_empty() {
        return Err(CurveError::Topology(
            "retained Bezier boundary loop requires nonempty fragments".to_owned(),
        ));
    }
    for fragment in fragments {
        validate_retained_fragment_provenance(fragment, policy)?;
    }
    validate_retained_boundary_loop_connectivity(fragments, policy)
}

fn validate_retained_fragment_provenance(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<()> {
    match fragment {
        BezierSplitFragment2::Materialized { start, end, .. } => {
            if start.scalar().is_none() || end.scalar().is_none() {
                return Err(CurveError::Topology(
                    "retained materialized Bezier fragment must carry exact range boundaries"
                        .into(),
                ));
            }
            validate_retained_fragment_parameter_order(start, end, policy)
        }
        BezierSplitFragment2::RetainedBezier {
            start,
            end,
            source_curve,
            start_image,
            end_image,
            ..
        } => {
            validate_retained_fragment_parameter_order(start, end, policy)?;
            validate_retained_source_endpoint_image(
                start,
                source_curve,
                start_image.as_ref(),
                policy,
            )?;
            validate_retained_source_endpoint_image(end, source_curve, end_image.as_ref(), policy)
        }
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            validate_retained_fragment_parameter_order(
                fragment.range().start(),
                fragment.range().end(),
                policy,
            )?;
            match crate::BezierParallelFragment2::try_new(
                fragment.parallel().clone(),
                fragment.range().clone(),
                policy,
            )? {
                Classification::Decided(_) => Ok(()),
                Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                    "analytic parallel fragment validation remained uncertain: {reason:?}"
                ))),
            }
        }
        BezierSplitFragment2::AlgebraicChord(chord) => {
            if chord.policy() != *policy && chord.policy() != policy.strict_counterpart() {
                return Err(CurveError::Topology(
                    "algebraic chord was replayed under a different predicate policy".into(),
                ));
            }
            Ok(())
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => fragment.validate_policy(policy),
        BezierSplitFragment2::SelectedFiber(fragment) => {
            match fragment
                .range()
                .start()
                .cmp_by_refinement(fragment.range().end(), policy)?
            {
                Classification::Decided(std::cmp::Ordering::Less) => Ok(()),
                Classification::Decided(
                    std::cmp::Ordering::Equal | std::cmp::Ordering::Greater,
                ) => Err(CurveError::Topology(
                    "selected-fiber fragment range was not increasing".into(),
                )),
                Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                    "selected-fiber fragment range remained uncertain: {reason:?}"
                ))),
            }
        }
    }
}

fn validate_retained_fragment_parameter_order(
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<()> {
    match start.cmp_by_refinement(end, policy)? {
        Classification::Decided(std::cmp::Ordering::Less) => Ok(()),
        Classification::Decided(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => {
            Err(CurveError::Topology(
                "retained Bezier fragment range must be certified strictly increasing".into(),
            ))
        }
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "retained Bezier fragment range ordering is uncertain: {reason:?}"
        ))),
    }
}

fn validate_retained_source_endpoint_image(
    boundary: &BezierParameter2,
    source_curve: &BezierSubcurve2,
    image: Option<&crate::BezierAlgebraicEndpointImage2>,
    policy: &CurveContext,
) -> CurveResult<()> {
    match boundary {
        BezierParameter2::Exact(parameter) => {
            if image.is_some() {
                return Err(CurveError::Topology(
                    "retained exact endpoint must not carry algebraic endpoint image evidence"
                        .into(),
                ));
            }
            // Connectivity may use source identity without projecting a point.
            // Establish affine endpoint validity at admission, including the
            // nonzero denominator of a rational chart, before that shortcut.
            if let Classification::Uncertain(reason) =
                subcurve_point_at(source_curve, parameter.clone(), policy)
            {
                return Err(CurveError::Topology(format!(
                    "could not certify retained boundary exact endpoint from source curve: {reason:?}"
                )));
            }
        }
        BezierParameter2::Algebraic(parameter) => {
            let Some(image) = image else {
                return Err(CurveError::Topology(
                    "retained algebraic boundary must carry endpoint image evidence".into(),
                ));
            };
            if image.parameter() != parameter {
                return Err(CurveError::Topology(
                    "retained algebraic endpoint image parameter does not match boundary".into(),
                ));
            }
            if !image.is_exact() && !image.is_lazy_first_order() {
                return Err(CurveError::Topology(
                    "retained algebraic endpoint image must retain exact or replayable first-order source evidence".into(),
                ));
            }
            let Classification::Decided(expected) =
                crate::BezierAlgebraicEndpointImage2::from_source_curve(
                    source_curve,
                    parameter,
                    policy,
                )?
            else {
                return Err(CurveError::Topology(
                    "retained algebraic endpoint is not a certified finite source point".into(),
                ));
            };
            if !image.matches_required_source_evidence(&expected) {
                return Err(CurveError::Topology(
                    "retained algebraic endpoint image does not match retained source curve".into(),
                ));
            }
        }
    }
    Ok(())
}

fn validate_retained_boundary_loop_sources(
    arrangement_sources: &[CurveRegionFragmentSource2],
) -> CurveResult<()> {
    let mut indices = arrangement_sources
        .iter()
        .map(|source| source.arrangement_fragment_index())
        .collect::<Vec<_>>();
    indices.sort_unstable();
    if indices.windows(2).any(|window| window[0] == window[1]) {
        return Err(CurveError::Topology(
            "retained boundary loop source provenance must not reuse arrangement fragments"
                .to_owned(),
        ));
    }
    Ok(())
}

fn validate_retained_region_loops(boundary_loops: &[CurveRegionBoundaryLoop2]) -> CurveResult<()> {
    for (index, boundary_loop) in boundary_loops.iter().enumerate() {
        if boundary_loops[index + 1..].iter().any(|candidate| {
            boundary_loop.fragments == candidate.fragments
                && boundary_loop.arrangement_sources == candidate.arrangement_sources
        }) {
            return Err(CurveError::Topology(
                "Bezier region must not duplicate boundary loop evidence".to_owned(),
            ));
        }
    }
    validate_retained_region_arrangement_sources(boundary_loops)
}

fn validate_retained_region_arrangement_sources(
    boundary_loops: &[CurveRegionBoundaryLoop2],
) -> CurveResult<()> {
    let mut indices = Vec::new();
    for boundary_loop in boundary_loops {
        if let Some(sources) = boundary_loop.arrangement_sources() {
            indices.extend(
                sources
                    .iter()
                    .map(|source| source.arrangement_fragment_index()),
            );
        }
    }
    validate_unique_arrangement_source_indices(
        indices,
        "retained Bezier region boundary loops must not reuse arrangement source fragments",
    )
}

fn validate_retained_boundary_loop_connectivity(
    fragments: &[BezierSplitFragment2],
    policy: &CurveContext,
) -> CurveResult<()> {
    for (left, right) in fragments
        .iter()
        .zip(fragments.iter().cycle().skip(1))
        .take(fragments.len())
    {
        match curve_fragment_endpoints_equal(left, false, right, true, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Err(CurveError::Topology(
                    "retained Bezier boundary loop fragments must be endpoint-connected and closed"
                        .into(),
                ));
            }
            Classification::Uncertain(_) => {
                return Err(CurveError::Topology(
                    "retained Bezier boundary loop must carry certified endpoint connectivity evidence"
                        .into(),
                ));
            }
        }
    }
    Ok(())
}

/// Gives each certified loop vertex one exact representation where a
/// straight fragment meets another materialized fragment.
///
/// Arrangement connectivity proves that consecutive fragments meet, but the
/// incident curves can still carry different exact expressions for the vertex,
/// for example a conic cut at an irrational point beside a line whose endpoint
/// is the line/circle contact itself. A later operation pairing either curve
/// with a copy of the other must then re-prove that coincidence by a zero test
/// on nested surds. The line's endpoint is the contact representation, so the
/// curved neighbour adopts it as its endpoint control, which changes no value
/// and keeps its weights, interior controls and retained evidence. Only
/// strictly certified connectivity proves equality; an approximate join may
/// rest on a decided coincidence of unequal values and is left unchanged.
fn share_line_image_vertices(fragments: &mut [BezierSplitFragment2], policy: &CurveContext) {
    if policy.permits_approximate_512() || policy.is_edge_preview() {
        return;
    }
    fn is_line(fragment: &BezierSplitFragment2) -> bool {
        matches!(
            fragment,
            BezierSplitFragment2::Materialized {
                curve: BezierSubcurve2::Quadratic(curve),
                ..
            } if curve.retained_exact_line_image().is_some()
        )
    }
    fn endpoint(fragment: &BezierSplitFragment2, at_end: bool) -> Option<&Point2> {
        let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
            return None;
        };
        let (start, end) = match curve {
            BezierSubcurve2::Quadratic(curve) => (curve.start(), curve.end()),
            BezierSubcurve2::Cubic(curve) => (curve.start(), curve.end()),
            BezierSubcurve2::RationalQuadratic(curve) => (curve.start(), curve.end()),
            // General rational endpoints are projected, not stored points.
            BezierSubcurve2::Rational(_) => return None,
        };
        Some(if at_end { end } else { start })
    }
    fn with_endpoint(
        fragment: &BezierSplitFragment2,
        point: &Point2,
        at_end: bool,
    ) -> Option<BezierSplitFragment2> {
        let BezierSplitFragment2::Materialized { start, end, curve } = fragment else {
            return None;
        };
        let replace = |first: &Point2, last: &Point2| {
            if at_end {
                (first.clone(), point.clone())
            } else {
                (point.clone(), last.clone())
            }
        };
        let curve = match curve {
            BezierSubcurve2::Quadratic(curve) => {
                let (first, last) = replace(curve.start(), curve.end());
                if curve.retained_exact_line_image().is_some() {
                    if !curve.retained_parallel_line_tangent_contacts().is_empty() {
                        return None;
                    }
                    BezierSubcurve2::Quadratic(
                        QuadraticBezier2::with_retained_exact_line_image(
                            first,
                            curve.control().clone(),
                            last,
                        )
                        .ok()?,
                    )
                } else {
                    BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                        first,
                        curve.control().clone(),
                        last,
                    ))
                }
            }
            BezierSubcurve2::Cubic(curve) => {
                let (first, last) = replace(curve.start(), curve.end());
                let [_, first_control, second_control, _] = curve.control_points();
                BezierSubcurve2::Cubic(CubicBezier2::new(
                    first,
                    first_control.clone(),
                    second_control.clone(),
                    last,
                ))
            }
            BezierSubcurve2::RationalQuadratic(curve) => {
                let (first, last) = replace(curve.start(), curve.end());
                BezierSubcurve2::RationalQuadratic(curve.with_equal_endpoints(first, last))
            }
            BezierSubcurve2::Rational(_) => return None,
        };
        Some(BezierSplitFragment2::Materialized {
            start: start.clone(),
            end: end.clone(),
            curve,
        })
    }
    let count = fragments.len();
    if count < 2 {
        return;
    }
    for index in 0..count {
        let next = (index + 1) % count;
        let (Some(end), Some(start)) = (
            endpoint(&fragments[index], true).cloned(),
            endpoint(&fragments[next], false).cloned(),
        ) else {
            continue;
        };
        if end == start {
            continue;
        }
        // The curved side adopts the line's contact representation; between
        // two lines the later one adopts the earlier endpoint.
        let replaced = if is_line(&fragments[next]) && !is_line(&fragments[index]) {
            with_endpoint(&fragments[index], &start, true).map(|fragment| (index, fragment))
        } else if is_line(&fragments[index]) {
            with_endpoint(&fragments[next], &end, false).map(|fragment| (next, fragment))
        } else {
            None
        };
        if let Some((slot, fragment)) = replaced {
            fragments[slot] = fragment;
        }
    }
}

fn validate_retained_arrangement_chain_connectivity(
    graph: &BezierArrangementGraph2,
    fragment_indices: &[usize],
    policy: &CurveContext,
) -> CurveResult<()> {
    for (&left_index, &right_index) in fragment_indices
        .iter()
        .zip(fragment_indices.iter().cycle().skip(1))
        .take(fragment_indices.len())
    {
        let left = graph.fragments().get(left_index).ok_or_else(|| {
            CurveError::Topology("retained traversal references a missing graph fragment".into())
        })?;
        let right = graph.fragments().get(right_index).ok_or_else(|| {
            CurveError::Topology("retained traversal references a missing graph fragment".into())
        })?;
        if let (Some(left_vertex), Some(right_vertex)) =
            (left.end_topology_vertex(), right.start_topology_vertex())
        {
            if left_vertex == right_vertex {
                continue;
            }
            return Err(CurveError::Topology(
                "retained arrangement chain joins distinct certified topology vertices".into(),
            ));
        }

        match curve_fragment_endpoints_equal(left.fragment(), false, right.fragment(), true, policy)
        {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Err(CurveError::Topology(
                    "retained arrangement chain contains disconnected fragments".into(),
                ));
            }
            Classification::Uncertain(_) => {
                return Err(CurveError::Topology(
                    "retained arrangement chain endpoint connectivity is uncertified".into(),
                ));
            }
        }
    }
    Ok(())
}

pub(crate) fn curve_fragment_endpoint_point(
    fragment: &BezierSplitFragment2,
    start_endpoint: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<crate::CurvePoint2>>> {
    let point = match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => crate::CurvePoint2::from(
            if start_endpoint {
                curve.start()
            } else {
                curve.end()
            }
            .clone(),
        ),
        BezierSplitFragment2::RetainedBezier {
            reversed,
            start,
            end,
            source_curve,
            ..
        } => {
            let parameter = if start_endpoint != *reversed {
                start
            } else {
                end
            };
            let source = RationalBezier2::try_from_subcurve(source_curve)?;
            return crate::rational_bezier_general::exact_contact_point_evidence(
                &source, parameter, policy,
            )
            .map(|point| point.map(Some));
        }
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            let parameter = if start_endpoint != fragment.is_reversed() {
                fragment.range().start()
            } else {
                fragment.range().end()
            };
            return fragment
                .parallel()
                .point_evidence_on_regular_range(
                    &parameter.clone().into(),
                    &CurveParameterRange2::from_bezier_range(fragment.range().clone()),
                    policy,
                )
                .map(|point| point.map(Some));
        }
        BezierSplitFragment2::AlgebraicChord(chord) => if start_endpoint {
            chord.start()
        } else {
            chord.end()
        }
        .clone(),
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            return fragment.endpoint_point_evidence(start_endpoint, policy);
        }
        BezierSplitFragment2::SelectedFiber(fragment) => if start_endpoint {
            fragment.start_point()
        } else {
            fragment.end_point()
        }
        .clone(),
    };
    Ok(Classification::Decided(Some(point)))
}

/// Compares retained endpoints before projecting their coordinates. Source and
/// overlap identities are positive certificates; unrelated endpoints continue
/// through the same exact point predicate used by paths and curve queries.
pub(crate) fn curve_fragment_endpoints_equal(
    first: &BezierSplitFragment2,
    first_start: bool,
    second: &BezierSplitFragment2,
    second_start: bool,
    policy: &CurveContext,
) -> Classification<bool> {
    if let (
        BezierSplitFragment2::RetainedBezier {
            source_curve: first_curve,
            start: first_lower,
            end: first_upper,
            reversed: first_reversed,
            ..
        },
        BezierSplitFragment2::RetainedBezier {
            source_curve: second_curve,
            start: second_lower,
            end: second_upper,
            reversed: second_reversed,
            ..
        },
    ) = (first, second)
    {
        let first_parameter = if first_start != *first_reversed {
            first_lower
        } else {
            first_upper
        };
        let second_parameter = if second_start != *second_reversed {
            second_lower
        } else {
            second_upper
        };
        if first_parameter == second_parameter && first_curve == second_curve {
            return Classification::Decided(true);
        }
    }
    if let (
        BezierSplitFragment2::AlgebraicCuspSemicircle(first),
        BezierSplitFragment2::AlgebraicCuspSemicircle(second),
    ) = (first, second)
        && first.shares_endpoint_evidence(first_start, second, second_start)
    {
        return Classification::Decided(true);
    }
    if let (Some((first_curve, first_parameter)), Some((second_curve, second_parameter))) = (
        fragment_endpoint_analytic_source(first, first_start),
        fragment_endpoint_analytic_source(second, second_start),
    ) && first_parameter == second_parameter
        && first_curve.shares_parameterized_curve_evidence(&second_curve)
    {
        return Classification::Decided(true);
    }

    let first_point = curve_fragment_endpoint_point(first, first_start, policy);
    let second_point = curve_fragment_endpoint_point(second, second_start, policy);
    // A selected circle may certify a mapped chord contact without projecting
    // its own endpoint. Preserve that incidence before requiring both points.
    for (fragment, start, point) in [
        (first, first_start, &second_point),
        (second, second_start, &first_point),
    ] {
        if let (
            BezierSplitFragment2::AlgebraicCuspSemicircle(circle),
            Ok(Classification::Decided(Some(CurvePoint2(CurvePointData2::AlgebraicCuspChord(
                point,
            ))))),
        ) = (fragment, point)
            && circle.shares_endpoint_point_evidence(start, point)
        {
            return Classification::Decided(true);
        }
    }
    match (first_point, second_point) {
        (Ok(Classification::Decided(Some(first))), Ok(Classification::Decided(Some(second)))) => {
            first.same_point(&second, policy)
        }
        (Ok(Classification::Uncertain(reason)), _) | (_, Ok(Classification::Uncertain(reason))) => {
            Classification::Uncertain(reason)
        }
        _ => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn fragment_endpoint_analytic_source(
    fragment: &BezierSplitFragment2,
    start_endpoint: bool,
) -> Option<(BezierParallel2, CurveParameter2)> {
    match fragment {
        BezierSplitFragment2::RetainedBezier {
            source_curve,
            start,
            end,
            reversed,
            ..
        } => Some((
            retained_subcurve_parallel(source_curve, Real::zero()).ok()?,
            if start_endpoint != *reversed {
                start
            } else {
                end
            }
            .clone()
            .into(),
        )),
        BezierSplitFragment2::AnalyticParallel(fragment) => Some((
            fragment.parallel().clone(),
            if start_endpoint != fragment.is_reversed() {
                fragment.range().start()
            } else {
                fragment.range().end()
            }
            .clone()
            .into(),
        )),
        BezierSplitFragment2::SelectedFiber(fragment) => Some((
            fragment.parallel_carrier(),
            if start_endpoint != fragment.is_reversed() {
                fragment.range().start()
            } else {
                fragment.range().end()
            }
            .clone(),
        )),
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            fragment.endpoint_analytic_source(start_endpoint)
        }
        _ => None,
    }
}

fn retained_subcurve_parallel(
    source: &BezierSubcurve2,
    distance: Real,
) -> CurveResult<BezierParallel2> {
    match source {
        BezierSubcurve2::Quadratic(source) => source.parallel_left(distance),
        BezierSubcurve2::Cubic(source) => source.parallel_left(distance),
        BezierSubcurve2::RationalQuadratic(source) => source.parallel_left(distance),
        BezierSubcurve2::Rational(source) => source.parallel_left(distance),
    }
}

/// A normalized boundary can still be concave between its junctions. The
/// convex-offset shortcut needs a certificate for every intervening carrier,
/// in addition to the endpoint turn checks made during span assembly.
fn fragment_certifies_nonnegative_turn(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<bool> {
    match fragment {
        BezierSplitFragment2::AlgebraicChord(_) => Ok(true),
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            Ok(fragment.semicircle().is_clockwise() == fragment.is_reversed())
        }
        BezierSplitFragment2::Materialized { curve, .. } => retained_subcurve_parallel(
            curve,
            Real::zero(),
        )?
        .certifies_nonnegative_turn(&CurveParameterRange2::unit(), false, policy),
        BezierSplitFragment2::RetainedBezier {
            source_curve,
            reversed,
            start,
            end,
            ..
        } => retained_subcurve_parallel(source_curve, Real::zero())?.certifies_nonnegative_turn(
            &CurveParameterRange2::new_validated(start.clone().into(), end.clone().into()),
            *reversed,
            policy,
        ),
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            fragment.parallel().certifies_nonnegative_turn(
                &CurveParameterRange2::from_bezier_range(fragment.range().clone()),
                fragment.is_reversed(),
                policy,
            )
        }
        BezierSplitFragment2::SelectedFiber(fragment) => fragment
            .parallel_carrier()
            .certifies_nonnegative_turn(fragment.range(), fragment.is_reversed(), policy),
    }
}

fn curve_path_from_native_contour(contour: &Contour2) -> ExactCurveResult<CurvePath2> {
    let curves = contour
        .segments()
        .iter()
        .map(|segment| match segment {
            Segment2::Line(line) => crate::Curve2::from(line.clone()),
            Segment2::Arc(arc) => crate::Curve2::from(arc.clone()),
        })
        .collect();
    CurvePath2::try_new(curves)
}

fn native_region_from_curve_paths(
    paths: &[CurvePath2],
    roles: &[CurveRegionLoopRole],
    fill_rules: &[FillRule],
) -> CurveResult<Option<LineArcRegion2>> {
    if paths.len() != roles.len() || paths.len() != fill_rules.len() {
        return Err(CurveError::Topology(
            "native curve-path role and fill-rule counts must match".into(),
        ));
    }

    let mut material = Vec::new();
    let mut holes = Vec::new();
    for ((path, role), fill_rule) in paths.iter().zip(roles).zip(fill_rules) {
        let Some(segments) = path
            .curves()
            .iter()
            .map(|curve| match curve.geometry() {
                Some(CurveGeometry2::Line(line)) => Some(Segment2::Line(line.clone())),
                Some(CurveGeometry2::CircularArc(arc)) => Some(Segment2::Arc(arc.clone())),
                _ => None,
            })
            .collect::<Option<Vec<_>>>()
        else {
            return Ok(None);
        };
        // `Contour2` intentionally rejects a segment whose endpoints are the
        // same, while `CircularArc2` uses that exact endpoint topology for a
        // full circle. Such a circle is already represented losslessly by the
        // canonical rational-conic boundary above; it is merely ineligible
        // for the private line/arc specialization.
        if segments.iter().any(|segment| match segment {
            Segment2::Arc(arc) => arc.start() == arc.end(),
            Segment2::Line(_) => false,
        }) {
            return Ok(None);
        }
        let contour = Contour2::try_new_with_fill_rule(segments, *fill_rule)?;
        match role {
            CurveRegionLoopRole::Material => material.push(contour),
            CurveRegionLoopRole::Hole => holes.push(contour),
        }
    }
    Ok(Some(LineArcRegion2::new(material, holes)))
}

fn curve_region_promotion_error(cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(CurveOperation2::Construction, CurveFamily2::Line, cause)
}

fn arrange_unordered_native_segments_raw(
    source_segments: &[Segment2],
    fill_rule: FillRule,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegion2> {
    let rings = assemble_unordered_segment_rings(source_segments, policy).map_err(|reason| {
        let family = if source_segments
            .iter()
            .any(|segment| matches!(segment, Segment2::Arc(_)))
        {
            CurveFamily2::CircularArc
        } else {
            CurveFamily2::Line
        };
        ExactCurveError::blocked(CurveOperation2::Construction, family, reason)
    })?;
    let paths = rings
        .into_iter()
        .map(|ring| {
            CurvePath2::from_structurally_closed_curves(
                ring.into_iter()
                    .map(|segment| match segment {
                        Segment2::Line(line) => Curve2::from(line),
                        Segment2::Arc(arc) => Curve2::from(arc),
                    })
                    .collect(),
            )
        })
        .collect::<Vec<_>>();
    let mut raw = CurveRegion2::try_from_boundary_paths_raw(&paths, policy)?;
    if !raw.is_empty() {
        raw.data_mut_for_construction().state =
            CurveRegionState2::Authored(Arc::from(vec![fill_rule; paths.len()]));
    }
    raw.finish_construction(policy)
}

fn curve_region_edit_error(operation: CurveOperation2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(operation, CurveFamily2::Line, cause)
}

struct RetainedPreselectedArcFilletContact2 {
    replacement: Option<Vec<BezierSplitFragment2>>,
    source_parallel: BezierParallel2,
    source_parameter: CurveParameter2,
    /// Orientation of the source-arc tangent relative to the offset cell
    /// whose rational contact map selected the fillet center.
    source_direction: RealSign,
}

impl CurveRegion2 {
    fn data_mut_for_construction(&mut self) -> &mut CurveRegionData2 {
        if Arc::get_mut(&mut self.data).is_none() {
            assert!(
                self.data.boundary_loops.is_empty(),
                "nonempty CurveRegion2 construction must own its data"
            );
            let mut data = CurveRegionData2::new(Vec::new());
            data.certified_loop_roles = self.data.certified_loop_roles.clone();
            data.state = self.data.state.clone();
            self.data = Arc::new(data);
        }
        Arc::get_mut(&mut self.data).expect("CurveRegion2 construction data is uniquely owned")
    }

    /// Constructs an empty unified region.
    pub fn empty() -> Self {
        Self::default()
    }

    /// Arranges unordered exact line/arc segments through unified region topology.
    ///
    /// The input adapter only orders endpoint-disjoint closed walks. Interior
    /// contacts, overlaps, winding, face selection, and output roles are all
    /// decided by the same all-family arrangement used by Boolean and offset
    /// operations. The rule fills each assembled walk; nesting and composition
    /// across walks use parity. An empty collection produces the canonical
    /// empty region. Unresolved assembly or arrangement returns an exact blocker.
    /// Output counts and native views are available on the returned region.
    pub fn arrange_unordered_segments(
        source_segments: &[Segment2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            arrange_unordered_native_segments_raw(source_segments, fill_rule, attempt)
        })
    }

    /// Constructs a unified region directly from explicit native contour roles.
    ///
    /// The contour carrier is retained as the certified line/arc fast path, but
    /// the returned authoritative object is `CurveRegion2`. This is the direct
    /// migration constructor for callers that already know which contours are
    /// material and which are holes.
    pub fn try_from_native_contours(
        material_contours: Vec<Contour2>,
        hole_contours: Vec<Contour2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            // Region admission may discard authored subdivision vertices.
            // Compact certified codirected line runs before building carriers;
            // unresolved collinearity leaves the original contour intact.
            let compact = |contour: Contour2| match contour
                .merge_adjacent_collinear_lines(&CurveContext::STRICT)
            {
                Ok(Classification::Decided(compact)) => compact,
                _ => contour,
            };
            let material_contours = material_contours.into_iter().map(compact).collect();
            let hole_contours = hole_contours.into_iter().map(compact).collect();
            Self::try_from_native_contours_raw(material_contours, hole_contours, attempt)?
                .finish_construction(attempt)
        })
    }

    pub(crate) fn try_from_native_contours_raw(
        material_contours: Vec<Contour2>,
        hole_contours: Vec<Contour2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if material_contours.is_empty() && hole_contours.is_empty() {
            return Ok(Self::default());
        }
        let paths = material_contours
            .iter()
            .chain(&hole_contours)
            .map(curve_path_from_native_contour)
            .collect::<ExactCurveResult<Vec<_>>>()?;
        let roles = std::iter::repeat_n(CurveRegionLoopRole::Material, material_contours.len())
            .chain(std::iter::repeat_n(
                CurveRegionLoopRole::Hole,
                hole_contours.len(),
            ))
            .collect::<Vec<_>>();
        let fill_rules = material_contours
            .iter()
            .chain(&hole_contours)
            .map(Contour2::fill_rule)
            .collect::<Vec<_>>();
        let mut promoted = Self::try_from_boundary_paths_with_loop_semantics_raw(
            &paths,
            &roles,
            &fill_rules,
            policy,
        )?;
        promoted.data_mut_for_construction().line_image_region = PolicyClassificationCache::new();
        promoted
            .data
            .line_image_region
            .certify(Some(LineArcRegion2::new(material_contours, hole_contours)));
        Ok(promoted)
    }

    /// Materializes already-certified, filled-left affine-line loops without
    /// replaying path construction or loop nesting.
    ///
    /// This is the compact output boundary for the authoritative Boolean
    /// arrangement. The traversal has already certified closure, face side,
    /// and material/hole role; every input contour has already merged adjacent
    /// codirected line runs. Keeping this constructor private to the crate
    /// prevents unproved authored contours from bypassing ordinary validation.
    pub(crate) fn from_certified_oriented_line_contours(
        material_contours: Vec<Contour2>,
        hole_contours: Vec<Contour2>,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        if material_contours.is_empty() && hole_contours.is_empty() {
            return Ok(Self::default());
        }

        let mut boundary_loops =
            Vec::with_capacity(material_contours.len().saturating_add(hole_contours.len()));
        for contour in material_contours.iter().chain(&hole_contours) {
            let fragments = contour
                .segments()
                .iter()
                .map(|segment| {
                    let Segment2::Line(line) = segment else {
                        return Err(CurveError::Topology(
                            "certified affine-line Boolean output contains a nonlinear segment"
                                .into(),
                        ));
                    };
                    Ok(BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                            line.clone(),
                        )),
                    })
                })
                .collect::<CurveResult<Vec<_>>>()?;
            boundary_loops.push(CurveRegionBoundaryLoop2 {
                fragments,
                arrangement_sources: None,
                connectivity_policy: Some(policy.retained_object_policy()),
                curves: OnceLock::new(),
                rational_evaluators: OnceLock::new(),
                query_bounds: OnceLock::new(),
            });
        }

        let loop_count = boundary_loops.len();
        let roles = std::iter::repeat_n(CurveRegionLoopRole::Material, material_contours.len())
            .chain(std::iter::repeat_n(
                CurveRegionLoopRole::Hole,
                hole_contours.len(),
            ))
            .collect::<Arc<[_]>>();
        let mut data = CurveRegionData2::new(boundary_loops);
        data.certified_loop_roles = Some(roles);
        // The caller's arrangement already certified these oriented, merged
        // line contours. Preserve that proof when choosing the compact native
        // representation, so the next operation does not normalize again.
        data.state = CurveRegionState2::Normalized(policy.retained_object_policy());
        data.filled_side_is_left
            .certify(Arc::from(vec![true; loop_count]));
        data.line_image_region
            .certify(Some(LineArcRegion2::new(material_contours, hole_contours)));
        Ok(Self {
            data: Arc::new(data),
        })
    }

    /// Constructs a unified region whose native contours are all material.
    pub fn try_from_native_material_contours(
        material_contours: Vec<Contour2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        Self::try_from_native_contours(material_contours, Vec::new(), policy)
    }

    /// Constructs the exact regularized fill of native boundary contours.
    ///
    /// The explicit fill rule applies to the sum of signed winding across all
    /// contours, independent of their individual fill rules. `EvenOdd` gives
    /// nesting parity; `NonZero` adds equally oriented contours and cancels
    /// opposite traversals. Crossings, overlaps, and touching boundaries use
    /// the same arrangement as general boundary paths.
    pub fn try_from_native_boundary_contours(
        contours: &[Contour2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        let paths = contours
            .iter()
            .map(curve_path_from_native_contour)
            .collect::<ExactCurveResult<Vec<_>>>()?;
        Self::try_from_boundary_paths(&paths, fill_rule, policy)
    }

    /// Classifies native contours through the shared raw-loop nesting authority.
    ///
    /// Contours become exact Bezier/conic paths. The general path-intersection
    /// kernel checks pair contacts before containment assigns roles. The
    /// result supplies roles and signed areas without copying boundary
    /// provenance into a separate report.
    fn native_boundary_contour_nesting_raw(
        contours: &[Contour2],
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<NativeLoopNesting2>> {
        let paths = contours
            .iter()
            .map(curve_path_from_native_contour)
            .collect::<ExactCurveResult<Vec<_>>>()?;
        let region = Self::try_from_boundary_paths_raw(&paths, policy)?;
        region
            .native_loop_nesting_raw(policy)
            .map_err(curve_region_promotion_error)
    }

    pub(crate) fn try_from_line_arc_region_raw(
        region: &LineArcRegion2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        Self::try_from_native_contours_raw(
            region.material_contours().to_vec(),
            region.hole_contours().to_vec(),
            policy,
        )
    }

    /// Constructs a curved region with explicit material/hole and fill semantics.
    ///
    /// This is the canonical authored-loop constructor when nesting parity is
    /// not the intended topology—for example nested material islands or
    /// self-overlapping loops using non-zero winding. One role and fill rule
    /// must be supplied for every boundary path. Each filled loop contributes
    /// +1 for material or -1 for a hole; positive total depth selects the set.
    /// The arrangement certifies the interior side from exact winding;
    /// callers do not supply orientation hints. Construction returns the
    /// regularized boundary, with material on the left.
    pub fn try_from_boundary_paths_with_loop_semantics(
        paths: &[CurvePath2],
        roles: &[CurveRegionLoopRole],
        fill_rules: &[FillRule],
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            Self::try_from_boundary_paths_with_loop_semantics_raw(
                paths, roles, fill_rules, attempt,
            )?
            .finish_construction(attempt)
        })
    }

    pub(crate) fn try_from_boundary_paths_with_loop_semantics_raw(
        paths: &[CurvePath2],
        roles: &[CurveRegionLoopRole],
        fill_rules: &[FillRule],
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if paths.len() != roles.len() || paths.len() != fill_rules.len() {
            let family = paths
                .first()
                .map_or(CurveFamily2::Line, |path| path.curves()[0].family());
            return Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                family,
                CurveError::Topology(
                    "curved-region loop roles and fill rules must match boundary path count".into(),
                ),
            ));
        }
        let mut region = Self::try_from_boundary_paths_raw(paths, policy)?;
        {
            let data = region.data_mut_for_construction();
            data.certified_loop_roles = Some(Arc::from(roles));
            data.state = CurveRegionState2::Authored(Arc::from(fill_rules));
        }
        if let Some(native) = native_region_from_curve_paths(paths, roles, fill_rules)
            .map_err(curve_region_promotion_error)?
        {
            region.data.line_image_region.certify(Some(native));
        }
        Ok(region)
    }

    /// Constructs the exact regularized fill of closed boundary paths.
    ///
    /// The fill rule applies to the sum of signed winding across all paths.
    /// Thus equally oriented overlapping paths add under `NonZero`, while
    /// opposite traversals cancel. `EvenOdd` selects odd total winding.
    /// Reuses each path's cached boundary, including retained generated curves;
    /// construction removes canceled seams and orients material on the left.
    pub fn try_from_boundary_paths(
        paths: &[CurvePath2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            Self::regularize_boundary_paths_raw(paths, fill_rule, attempt)
                .map_err(|error| error.with_operation(CurveOperation2::Construction))
        })
    }

    /// Authored loops are construction inputs. Publish only their regularized
    /// filled boundary, with canceled seams removed and ownership retained.
    fn finish_construction(self, policy: &CurveContext) -> ExactCurveResult<Self> {
        self.regularized_region_raw(policy)
            .map_err(|error| error.with_operation(CurveOperation2::Construction))
    }

    pub(crate) fn try_from_boundary_paths_raw(
        paths: &[CurvePath2],
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let mut boundary_loops = Vec::with_capacity(paths.len());
        let mut next_arrangement_fragment_index = 0;
        for path in paths {
            let mut boundary_loop = match path
                .boundary_loop_with_policy(policy)
                .map_err(|error| error.with_operation(CurveOperation2::Construction))?
            {
                Classification::Decided(boundary) => boundary.clone(),
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Construction,
                        path.curves()[0].family(),
                        reason,
                    ));
                }
            };
            boundary_loop.arrangement_sources = Some(
                (0..boundary_loop.len())
                    .map(|_| {
                        let index = next_arrangement_fragment_index;
                        next_arrangement_fragment_index += 1;
                        CurveRegionFragmentSource2::new(index, index, 0)
                    })
                    .collect(),
            );
            boundary_loops.push(boundary_loop);
        }
        Self::new(boundary_loops).map_err(|cause| {
            let family = paths
                .first()
                .map_or(CurveFamily2::Line, |path| path.curves()[0].family());
            ExactCurveError::invalid(CurveOperation2::Construction, family, cause)
        })
    }

    /// Applies a nonsingular exact planar affine transform to every retained
    /// carrier while preserving certified arrangement connectivity.
    #[allow(clippy::too_many_arguments)]
    pub fn transform_affine(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
        tx: &Real,
        ty: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            self.transform_affine_raw(m00, m01, m10, m11, tx, ty, attempt)
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn transform_affine_raw(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
        tx: &Real,
        ty: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        // The identity keeps every retained carrier and its evidence; wrapping
        // them as transformed images would only hide that shared identity.
        let structurally_zero =
            |value: &Real| value.zero_status() == hyperreal::ZeroKnowledge::Zero;
        if structurally_zero(&(m00 - Real::one()))
            && structurally_zero(&(m11 - Real::one()))
            && [m01, m10, tx, ty].into_iter().all(structurally_zero)
        {
            return Ok(self.clone());
        }
        let determinant = m00 * m11 - m01 * m10;
        let orientation_reversing = match real_sign(&determinant, policy) {
            Some(RealSign::Positive) => false,
            Some(RealSign::Negative) => true,
            Some(RealSign::Zero) => {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Transformation,
                    CurveFamily2::RationalBezier,
                    CurveError::InvalidAffineTransform,
                ));
            }
            None => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Transformation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::RealSign,
                ));
            }
        };

        let mut loops = Vec::with_capacity(self.data.boundary_loops.len());
        // A regularized face walk is canonically filled-left. Reflection
        // reverses both ambient orientation and every transformed fragment;
        // reverse each complete loop afterward so the same filled-left
        // topology certificate remains true instead of becoming stale
        // filled-right metadata.
        let retained_regularized_topology = self.has_regularized_filled_left_topology(policy);
        let reverse_canonical_loops = orientation_reversing && retained_regularized_topology;
        let similarity = std::cell::OnceCell::new();
        let mut semicircle_similarity_cache =
            BezierAlgebraicCuspSemicircleSimilarityCache2::default();
        for boundary in &self.data.boundary_loops {
            let mut fragments = boundary
                .fragments()
                .iter()
                .map(|fragment| {
                    transform_retained_region_fragment(
                        fragment,
                        m00,
                        m01,
                        m10,
                        m11,
                        tx,
                        ty,
                        &similarity,
                        &mut semicircle_similarity_cache,
                        policy,
                    )
                })
                .collect::<ExactCurveResult<Vec<_>>>()?;
            if reverse_canonical_loops {
                fragments = fragments
                    .into_iter()
                    .rev()
                    .map(|fragment| fragment.reversed().map_err(affine_region_error))
                    .collect::<ExactCurveResult<Vec<_>>>()?;
            }
            let boundary = match boundary.arrangement_sources() {
                Some(sources) => {
                    let sources = if reverse_canonical_loops {
                        sources.iter().rev().cloned().collect()
                    } else {
                        sources.to_vec()
                    };
                    CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                        fragments, sources, policy,
                    )
                }
                None => CurveRegionBoundaryLoop2::new(fragments, policy),
            }
            .map_err(affine_region_error)?;
            loops.push(boundary);
        }
        let mut transformed = Self::new(loops).map_err(affine_region_error)?;
        {
            let data = transformed.data_mut_for_construction();
            data.certified_loop_roles = self.data.certified_loop_roles.clone();
            data.state = if retained_regularized_topology {
                CurveRegionState2::Normalized(policy.retained_object_policy())
            } else {
                match &self.data.state {
                    CurveRegionState2::Authored(rules) => {
                        CurveRegionState2::Authored(rules.clone())
                    }
                    CurveRegionState2::Raw | CurveRegionState2::Normalized(_) => {
                        CurveRegionState2::Raw
                    }
                }
            };
        }
        let sides = match self
            .filled_side_is_left_raw(policy)
            .map_err(affine_region_error)?
        {
            Classification::Decided(sides) => sides
                .iter()
                .map(|side| {
                    if orientation_reversing != reverse_canonical_loops {
                        !side
                    } else {
                        *side
                    }
                })
                .collect(),
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Transformation,
                    CurveFamily2::RationalBezier,
                    reason,
                ));
            }
        };
        let transformed = transformed
            .with_certified_filled_side_is_left(sides)
            .map_err(affine_region_error)?;
        if let Classification::Decided(region) = transformed
            .certified_line_image_region(policy)
            .map_err(affine_region_error)?
        {
            transformed.data.line_image_region.certify(Some(region));
        }
        Ok(transformed)
    }

    /// Validates raw boundary collections for internal normalization.
    pub(crate) fn new(boundary_loops: Vec<CurveRegionBoundaryLoop2>) -> CurveResult<Self> {
        if boundary_loops.is_empty() {
            return Ok(Self::default());
        }
        validate_retained_region_loops(&boundary_loops)?;
        Ok(Self::from_certified_boundary_loops(boundary_loops))
    }

    /// Attaches authored role, fill, and interior-side hints before normalization.
    ///
    /// These hints retain procedural carrier semantics without requiring a
    /// represented Green integral. They do not certify a regularized boundary;
    /// public path admission normalizes the resulting internal region.
    pub(crate) fn try_new_with_loop_topology(
        boundary_loops: Vec<CurveRegionBoundaryLoop2>,
        roles: Vec<CurveRegionLoopRole>,
        fill_rules: Vec<FillRule>,
        interior_sides: Vec<CurveBoundaryInteriorSide2>,
    ) -> CurveResult<Self> {
        let loop_count = boundary_loops.len();
        if roles.len() != loop_count
            || fill_rules.len() != loop_count
            || interior_sides.len() != loop_count
        {
            return Err(CurveError::Topology(
                "retained curved-region loop topology must match the boundary-loop count".into(),
            ));
        }
        let mut region = Self::new(boundary_loops)?;
        {
            let data = region.data_mut_for_construction();
            data.certified_loop_roles = Some(Arc::from(roles));
            data.state = CurveRegionState2::Authored(Arc::from(fill_rules));
        }
        region.with_certified_filled_side_is_left(
            interior_sides
                .into_iter()
                .map(|side| side == CurveBoundaryInteriorSide2::Left)
                .collect(),
        )
    }

    fn from_certified_boundary_loops(boundary_loops: Vec<CurveRegionBoundaryLoop2>) -> Self {
        if boundary_loops.is_empty() {
            return Self::default();
        }
        Self {
            data: Arc::new(CurveRegionData2::new(boundary_loops)),
        }
    }

    pub(crate) fn with_certified_filled_side_is_left(
        self,
        filled_side_is_left: Vec<bool>,
    ) -> CurveResult<Self> {
        if filled_side_is_left.len() != self.data.boundary_loops.len() {
            return Err(CurveError::Topology(
                "curved-region filled-side evidence must match the boundary-loop count".into(),
            ));
        }
        self.data
            .filled_side_is_left
            .certify(Arc::from(filled_side_is_left));
        Ok(self)
    }

    /// Publishes regularized filled-left topology with its decision requirement.
    ///
    /// An authoritative filled-left face walk is the general producer. Narrow
    /// exact geometric proofs, such as the one-turn cardinal convex parallel
    /// certificate, may publish the same fact. Unlike authored boundary
    /// provenance, this marker certifies that every retained chain is a
    /// noncrossing regularized boundary. Expensive exact nesting may therefore
    /// be deferred until a caller actually needs loop roles. A consumed
    /// APPROXIMATE_512 terminal remains a dependency of this topology;
    /// requesting that policy alone does not weaken an exact certificate.
    pub(crate) fn with_regularized_filled_left_topology(
        mut self,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        if self.data.boundary_loops.iter().any(|boundary_loop| {
            !boundary_loop.has_arrangement_sources() || boundary_loop.is_empty()
        }) {
            return Err(CurveError::Topology(
                "regularized filled-left topology requires arrangement provenance".into(),
            ));
        }
        let loop_count = self.data.boundary_loops.len();
        let retained =
            policy.retained_object_policy_with_dependencies(self.data.normalized_policy());
        let data = self.data_mut_for_construction();
        data.filled_side_is_left
            .certify(Arc::from(vec![true; loop_count]));
        if loop_count == 1 && data.certified_loop_roles.is_none() {
            data.certified_loop_roles = Some(shared_all_material_curve_region_loop_roles(1));
        }
        // Filled-left normalized loops have winding 0 or +/-1 everywhere, so
        // every per-loop fill rule selects the same set. Authored rules are
        // construction input and are not retained on the normalized result.
        data.state = CurveRegionState2::Normalized(retained);
        Ok(self)
    }

    pub(crate) fn has_regularized_filled_left_topology(&self, policy: &CurveContext) -> bool {
        let Some(retained) = self.data.normalized_policy() else {
            return false;
        };
        if !policy.accepts_retained_policy(retained) {
            return false;
        }
        if retained != retained.strict_counterpart() {
            if !policy.permits_approximate_512() {
                return false;
            }
            policy.observe_approximate_512();
        }
        true
    }

    /// Shares a successful, certified normalization of immutable authored loops.
    /// The certificate belongs to the operation from this input to its output:
    /// even an exact empty output does not certify approximate input decisions.
    pub(crate) fn resolve_regularization(
        &self,
        policy: &CurveContext,
        normalize: impl FnOnce() -> ExactCurveResult<Self>,
    ) -> ExactCurveResult<Self> {
        if !policy.is_edge_preview()
            && let Some(region) = self.data.certified_regularization.get()
        {
            return Ok(region.clone());
        }
        let outcome = resolve_certified_value(policy, |_| normalize());
        let region = outcome.value?;
        if outcome.certainty == CurveCertainty::Certified && !policy.is_edge_preview() {
            debug_assert!(!Arc::ptr_eq(&self.data, &region.data));
            let _ = self.data.certified_regularization.set(region);
            return Ok(self.data.certified_regularization.get().unwrap().clone());
        }
        Ok(region)
    }

    pub(crate) fn with_certified_loop_roles(
        mut self,
        roles: Vec<CurveRegionLoopRole>,
    ) -> CurveResult<Self> {
        if roles.len() != self.data.boundary_loops.len() {
            return Err(CurveError::Topology(
                "curved-region loop roles must match the boundary-loop count".into(),
            ));
        }
        let data = self.data_mut_for_construction();
        data.certified_loop_roles = Some(shared_curve_region_loop_roles(roles));
        Ok(self)
    }

    pub(crate) fn with_certified_all_material_loop_roles(
        mut self,
        role_count: usize,
    ) -> CurveResult<Self> {
        if role_count != self.data.boundary_loops.len() {
            return Err(CurveError::Topology(
                "curved-region material roles must match the boundary-loop count".into(),
            ));
        }
        self.data_mut_for_construction().certified_loop_roles =
            Some(shared_all_material_curve_region_loop_roles(role_count));
        Ok(self)
    }

    /// Publishes all-material roles when conservative loop boxes prove that
    /// no retained boundary can nest another one.
    ///
    /// This is an exact topology certificate, not a sampled classification:
    /// nested closed loops necessarily have overlapping outer boxes. It is
    /// especially useful immediately after a filled-left face walk splits a
    /// self-contacting procedural loop into disjoint material components.
    pub(crate) fn with_pairwise_disjoint_material_loop_roles(
        mut self,
        policy: &CurveContext,
    ) -> Self {
        if self.data.certified_loop_roles.is_none()
            && self.data.boundary_loops.len() > 1
            && policy.strict_predicate_pass(|| {
                retained_loops_have_pairwise_disjoint_bounds(&self.data.boundary_loops, policy)
            }) == Classification::Decided(true)
        {
            let role_count = self.data.boundary_loops.len();
            self.data_mut_for_construction().certified_loop_roles =
                Some(shared_all_material_curve_region_loop_roles(role_count));
        }
        self
    }

    pub fn filled_side_is_left(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<&[bool]>>> {
        resolve_certified_operation(policy, |attempt| self.filled_side_is_left_raw(attempt))
    }

    pub(crate) fn filled_side_is_left_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<&[bool]>> {
        let mut rational_quadratic_cache = RationalQuadraticAreaIntegralCache::default();
        self.filled_side_is_left_with_area_cache(policy, &mut rational_quadratic_cache)
    }

    pub(crate) fn filled_side_is_left_with_area_cache(
        &self,
        policy: &CurveContext,
        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Classification<&[bool]>> {
        resolve_cached_classification(&self.data.filled_side_is_left, policy, |attempt| {
            self.compute_filled_side_is_left_with_area_cache(attempt, rational_quadratic_cache)
        })
        .map(|classification| classification.map(AsRef::as_ref))
    }

    fn compute_filled_side_is_left_with_area_cache(
        &self,
        policy: &CurveContext,
        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Classification<Arc<[bool]>>> {
        if let Some(roles) = self.data.certified_loop_roles.as_deref() {
            let mut signed_areas = Vec::with_capacity(self.data.boundary_loops.len());
            for boundary_loop in &self.data.boundary_loops {
                match boundary_loop.signed_area_with_cache(policy, rational_quadratic_cache)? {
                    Classification::Decided(Some(area)) => signed_areas.push(area),
                    Classification::Decided(None) | Classification::Uncertain(_) => {
                        signed_areas.clear();
                        break;
                    }
                }
            }
            if signed_areas.len() == self.data.boundary_loops.len() {
                return filled_sides_from_roles_and_areas(roles, &signed_areas, policy)
                    .map(|sides| Classification::Decided(Arc::from(sides)));
            }
        }
        if self.data.boundary_loops.len() == 1 {
            match self.data.boundary_loops[0]
                .signed_area_with_cache(policy, rational_quadratic_cache)?
            {
                Classification::Decided(Some(area)) => {
                    return Ok(match real_sign(&area, policy) {
                        Some(RealSign::Positive) => {
                            Classification::Decided(Arc::from([true].as_slice()))
                        }
                        Some(RealSign::Negative) => {
                            Classification::Decided(Arc::from([false].as_slice()))
                        }
                        Some(RealSign::Zero) => {
                            Classification::Uncertain(UncertaintyReason::Boundary)
                        }
                        None => Classification::Uncertain(UncertaintyReason::RealSign),
                    });
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
        }

        match self.native_loop_nesting_raw(policy)? {
            Classification::Decided(evidence) => {
                return filled_sides_from_roles_and_areas(
                    &evidence.roles,
                    &evidence.signed_areas,
                    policy,
                )
                .map(|sides| Classification::Decided(Arc::from(sides)));
            }
            Classification::Uncertain(UncertaintyReason::Unsupported) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        match self.line_image_roles_and_contours_raw(policy)? {
            Classification::Decided((roles, contours)) => {
                let mut areas = Vec::with_capacity(contours.len());
                for contour in &contours {
                    let Some(area) = contour.signed_area()? else {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    };
                    areas.push(area);
                }
                filled_sides_from_roles_and_areas(&roles, &areas, policy)
                    .map(|sides| Classification::Decided(Arc::from(sides)))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(crate) fn from_certified_arrangement_traversal(
        graph: &BezierArrangementGraph2,
        traversal: &BezierArrangementTraversal2,
        policy: &CurveContext,
    ) -> Classification<Self> {
        Self::from_arrangement_traversal_raw(graph, traversal, policy, false)
    }

    fn from_arrangement_traversal_raw(
        graph: &BezierArrangementGraph2,
        traversal: &BezierArrangementTraversal2,
        policy: &CurveContext,
        validate: bool,
    ) -> Classification<Self> {
        let validation_policy = validate.then_some(policy);
        let mut loops = Vec::with_capacity(traversal.chains().len());
        for chain in traversal.chains() {
            if !chain.is_closed() {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }

            let mut fragments = Vec::with_capacity(chain.len());
            let mut arrangement_sources = Vec::with_capacity(chain.len());
            for index in chain.fragment_indices() {
                let Some(fragment) = graph.fragments().get(*index) else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                match fragment.fragment() {
                    BezierSplitFragment2::Materialized { .. }
                    | BezierSplitFragment2::RetainedBezier { .. }
                    | BezierSplitFragment2::AnalyticParallel(_)
                    | BezierSplitFragment2::AlgebraicChord(_) => {
                        fragments.push(fragment.fragment().clone());
                    }
                    BezierSplitFragment2::AlgebraicCuspSemicircle(circle) => {
                        // Authored tangent bits are valid only while the
                        // original input-loop adjacency is intact. An
                        // arrangement traversal may reconnect the endpoint to
                        // another carrier, so never publish that transient
                        // certificate into a later Boolean or offset.
                        fragments.push(BezierSplitFragment2::AlgebraicCuspSemicircle(
                            circle.clone().without_certified_tangent_endpoints(),
                        ));
                    }
                    BezierSplitFragment2::SelectedFiber(_) => {
                        fragments.push(fragment.fragment().clone());
                    }
                }
                arrangement_sources.push(CurveRegionFragmentSource2::new(
                    *index,
                    fragment.source_curve_index(),
                    fragment.source_fragment_index(),
                ));
            }
            if let Some(policy) = validation_policy
                && validate_retained_arrangement_chain_connectivity(
                    graph,
                    chain.fragment_indices(),
                    policy,
                )
                .is_err()
            {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            let loop_ = if let Some(policy) = validation_policy {
                match CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                    fragments,
                    arrangement_sources,
                    policy,
                ) {
                    Ok(loop_) => loop_,
                    Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
                }
            } else {
                CurveRegionBoundaryLoop2::from_certified_arrangement_chain(
                    fragments,
                    arrangement_sources,
                    policy,
                )
            };
            loops.push(loop_);
        }

        if validation_policy.is_some() {
            match Self::new(loops) {
                Ok(region) => Classification::Decided(region),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        } else {
            Classification::Decided(Self::from_certified_boundary_loops(loops))
        }
    }

    /// Lowers retained exact line images and assigns roles through the
    /// authoritative all-family nesting kernel.
    ///
    /// Every retained fragment must either be a materialized polynomial Bezier
    /// that is exactly a degree elevation of its endpoint line segment, or an
    /// algebraic endpoint-image carrier whose contributed endpoints are exact
    /// rational point witnesses. The method lowers those loops to native line
    /// contours, validates every potentially intersecting pair through the
    /// all-family curve kernel, and assigns even-odd nesting roles with the
    /// authoritative curved-loop containment classifier. It rejects conics,
    /// nonlinear Bezier arcs, algebraic endpoint-image carriers without exact
    /// rational endpoints, unresolved fragments, boundary-touching loops, and
    /// uncertain predicate signs.
    fn line_image_roles_and_contours_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Vec<CurveRegionLoopRole>, Vec<Contour2>)>> {
        let mut contours = Vec::with_capacity(self.data.boundary_loops.len());
        for boundary_loop in &self.data.boundary_loops {
            let contour = match retained_line_loop_to_contour(boundary_loop, policy)? {
                Classification::Decided(contour) => contour,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            contours.push(contour);
        }

        let nesting = match Self::native_boundary_contour_nesting_raw(&contours, policy) {
            Ok(Classification::Decided(evidence)) => evidence,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            Err(ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
        };
        Ok(Classification::Decided((nesting.roles, contours)))
    }

    /// Assigns material/hole roles by exact curved-loop nesting.
    ///
    /// Each retained loop must be fully native and have a nonzero implemented
    /// signed area. Potentially overlapping loop pairs first pass through the
    /// all-family exact path-intersection kernel; any contact or overlap blocks
    /// nesting. The area is used only to reject degenerate/unsupported loops;
    /// role parity comes from exact containment depth. This makes
    /// same-orientation nested nonlinear loops classify as material/hole by
    /// topology instead of by their authored orientation.
    fn native_loop_nesting_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<NativeLoopNesting2>> {
        let Some(native_loops) = self.native_boundary_loops() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let native_bounds = self.native_boundary_bounds(policy);
        let mut native_paths = vec![None; native_loops.len()];

        // Role assignment is valid only for disjoint or strictly nested
        // loops. Validate every potentially overlapping pair through the same
        // all-family curve interaction kernel used by path and region
        // topology. A decided contact or overlap dominates an incomplete
        // predicate because either already proves that nesting is not valid.
        for first_index in 0..native_loops.len() {
            for second_index in first_index + 1..native_loops.len() {
                if native_bounds.is_some_and(|bounds| {
                    matches!(
                        bounds[first_index].overlaps(&bounds[second_index], policy),
                        Classification::Decided(false)
                    )
                }) {
                    continue;
                }
                for index in [first_index, second_index] {
                    if native_paths[index].is_some() {
                        continue;
                    }
                    let curves = native_loops[index]
                        .fragments()
                        .iter()
                        .cloned()
                        .map(Curve2::from)
                        .collect();
                    match CurvePath2::try_new_raw(curves, policy) {
                        Ok(path) => native_paths[index] = Some(path),
                        Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
                        Err(ExactCurveError::Blocked(blocker)) => {
                            return Ok(Classification::Uncertain(blocker.reason()));
                        }
                    }
                }
                let intersections = match native_paths[first_index]
                    .as_ref()
                    .expect("an overlapping first loop has one exact path")
                    .intersect_path_raw(
                        native_paths[second_index]
                            .as_ref()
                            .expect("an overlapping second loop has one exact path"),
                        policy,
                    ) {
                    Ok(intersections) => intersections,
                    Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
                    Err(ExactCurveError::Blocked(blocker)) => {
                        return Ok(Classification::Uncertain(blocker.reason()));
                    }
                };
                if !intersections.contacts().is_empty() || !intersections.overlaps().is_empty() {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                if let Some(blocker) = intersections.blockers().first() {
                    let reason = match blocker.blocker().kind() {
                        CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
                        CurveIntersectionPairBlockerKind2::IncompleteReplay => {
                            UncertaintyReason::Predicate
                        }
                        CurveIntersectionPairBlockerKind2::SharedComponent => {
                            UncertaintyReason::Boundary
                        }
                    };
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }

        let mut sample_points = Vec::with_capacity(self.data.boundary_loops.len());
        let mut signed_areas = Vec::with_capacity(self.data.boundary_loops.len());
        for native_loop in native_loops {
            let area = match native_loop.signed_area_raw(policy)? {
                Classification::Decided(Some(area)) => area,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match real_sign(&area, policy) {
                Some(RealSign::Positive | RealSign::Negative) => {}
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            let sample = match native_loop_sample_point(native_loop, policy) {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            sample_points.push(sample);
            signed_areas.push(area);
        }

        let mut roles = Vec::with_capacity(native_loops.len());
        for (candidate_index, sample) in sample_points.iter().enumerate() {
            let mut depth = 0_usize;
            for (container_index, container) in native_loops.iter().enumerate() {
                if candidate_index == container_index {
                    continue;
                }
                if native_bounds.is_some_and(|bounds| {
                    matches!(
                        bounds[container_index].contains_point(sample, policy),
                        Classification::Decided(false)
                    )
                }) {
                    continue;
                }
                match classify_point_against_native_loop_after_bounds(container, sample, policy)? {
                    Classification::Decided(ContourPointLocation::Inside) => depth += 1,
                    Classification::Decided(ContourPointLocation::Outside) => {}
                    Classification::Decided(ContourPointLocation::Boundary) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            roles.push(if depth.is_multiple_of(2) {
                CurveRegionLoopRole::Material
            } else {
                CurveRegionLoopRole::Hole
            });
        }

        // Check source identity on its authoritative owner instead of cloning
        // provenance into a separate nesting report and validating that copy.
        validate_retained_region_arrangement_sources(&self.data.boundary_loops)?;
        Ok(Classification::Decided(NativeLoopNesting2 {
            roles,
            signed_areas,
        }))
    }

    /// Returns one exact material/hole role per retained loop.
    ///
    /// Native curves and exact retained line images keep their direct nesting
    /// fast paths. Already-regularized higher carriers retain their point
    /// evidence and enter the Boolean carrier-pair probe. Any genuinely
    /// unsupported carrier remains explicit uncertainty; authored orientation
    /// never overrides a topology blocker.
    pub fn loop_roles(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Vec<CurveRegionLoopRole>>>> {
        resolve_certified_operation(policy, |attempt| self.loop_roles_raw(attempt))
    }

    pub(crate) fn loop_roles_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveRegionLoopRole>>> {
        if let Some(roles) = &self.data.certified_loop_roles {
            return Ok(Classification::Decided(roles.to_vec()));
        }
        if self.data.boundary_loops.len() == 1 {
            return Ok(Classification::Decided(vec![CurveRegionLoopRole::Material]));
        }
        if retained_loops_have_pairwise_disjoint_bounds(&self.data.boundary_loops, policy)
            == Classification::Decided(true)
        {
            return Ok(Classification::Decided(vec![
                CurveRegionLoopRole::Material;
                self.data.boundary_loops.len()
            ]));
        }
        // The authoritative face walk has already certified that every output
        // chain is a noncrossing filled-left boundary. Replaying all-family
        // pair intersections here is both redundant and weaker: compact
        // procedural conics can retain the face certificate even when a fresh
        // standalone path-construction predicate is not representable. Use the
        // certificate-aware retained classifier directly.
        if self.has_regularized_filled_left_topology(policy) {
            return self.regularized_retained_loop_roles_raw(policy);
        }
        match self.native_loop_nesting_raw(policy)? {
            Classification::Decided(evidence) => {
                return Ok(Classification::Decided(evidence.roles));
            }
            Classification::Uncertain(UncertaintyReason::Unsupported) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match self.line_image_roles_and_contours_raw(policy)? {
            Classification::Decided((roles, _)) => Ok(Classification::Decided(roles)),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Assigns roles to already-regularized retained boundary loops by exact nesting.
    ///
    /// The retained regularization certificate guarantees that distinct
    /// output chains are noncrossing simple boundaries with fill on their
    /// left. Exact interior point evidence from one retained fragment is
    /// therefore inside exactly the loops that contain that complete
    /// boundary. Multi-field evidence stays retained and enters the Boolean
    /// carrier-pair probe. Nesting parity assigns material and hole roles
    /// without requiring a Green integral for procedural analytic parallels.
    pub(crate) fn regularized_retained_loop_roles_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveRegionLoopRole>>> {
        match self.data.boundary_loops.len() {
            0 => return Ok(Classification::Decided(Vec::new())),
            1 => {
                return Ok(Classification::Decided(vec![CurveRegionLoopRole::Material]));
            }
            _ => {}
        }
        let mut samples = Vec::with_capacity(self.data.boundary_loops.len());
        let mut bounds = Vec::with_capacity(self.data.boundary_loops.len());
        for boundary_loop in &self.data.boundary_loops {
            match retained_loop_sample_point_evidence(boundary_loop, policy)? {
                Classification::Decided(point) => samples.push(point),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            bounds.push(match retained_loop_query_bounds(boundary_loop, policy) {
                Classification::Decided(bounds) => Some(bounds),
                Classification::Uncertain(_) => None,
            });
        }

        let mut roles = Vec::with_capacity(self.data.boundary_loops.len());
        for (candidate_index, sample) in samples.iter().enumerate() {
            let mut depth = 0_usize;
            for container_index in 0..self.data.boundary_loops.len() {
                if candidate_index == container_index {
                    continue;
                }
                if sample.coordinates().is_some_and(|point| {
                    bounds[container_index].as_ref().is_some_and(|bounds| {
                        matches!(
                            bounds.contains_point(point, policy),
                            Classification::Decided(false)
                        )
                    })
                }) {
                    continue;
                }
                match classify_point_evidence_against_retained_loop(
                    self,
                    container_index,
                    sample,
                    policy,
                )? {
                    Classification::Decided(ContourPointLocation::Inside) => depth += 1,
                    Classification::Decided(ContourPointLocation::Outside) => {}
                    Classification::Decided(ContourPointLocation::Boundary) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            roles.push(if depth.is_multiple_of(2) {
                CurveRegionLoopRole::Material
            } else {
                CurveRegionLoopRole::Hole
            });
        }
        Ok(Classification::Decided(roles))
    }

    /// Returns the number of material and hole loops in authoritative topology.
    ///
    /// The tuple is `(material, holes)`. Role classification follows the same
    /// exact retained-curve path as [`CurveRegion2::loop_roles`]; no native
    /// projection is required.
    pub fn loop_role_counts(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<(usize, usize)>>> {
        resolve_certified_operation(policy, |attempt| self.loop_role_counts_raw(attempt))
    }

    pub(crate) fn loop_role_counts_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(usize, usize)>> {
        self.loop_roles_raw(policy).map(|roles| {
            roles.map(|roles| {
                let material = roles
                    .iter()
                    .filter(|role| **role == CurveRegionLoopRole::Material)
                    .count();
                (material, roles.len() - material)
            })
        })
    }

    /// Groups retained material loops with their exact owned hole loops.
    ///
    /// Roles come from [`Self::loop_roles`]. Each hole contributes exact
    /// retained representative evidence which is classified against material
    /// carriers without materializing multi-field coordinates.
    pub fn boundary_profiles(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Vec<CurveRegionProfile2<'_>>>>> {
        resolve_certified_operation(policy, |attempt| self.boundary_profiles_raw(attempt))
    }

    /// Returns each regularized material exterior with its owned holes.
    ///
    /// Islands inside holes become separate components. Components may touch
    /// at isolated boundary points. Each result has its material loop first,
    /// followed by its holes, and every boundary has material on its left.
    /// Selected curves and certified connectivity are retained directly;
    /// decomposition does not reconstruct endpoints or intersect the boundary
    /// again after normalization and exact hole ownership have been decided.
    pub fn material_components(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            let normalized = self
                .regularized_region_raw(attempt)
                .map_err(|error| error.with_operation(CurveOperation2::Construction))?;
            let profiles = match normalized
                .boundary_profiles_raw(attempt)
                .map_err(curve_region_promotion_error)?
            {
                Classification::Decided(profiles) => profiles,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Construction,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            };
            if profiles.len() == 1
                && profiles[0].material_loop_index == 0
                && profiles[0]
                    .hole_loop_indices
                    .iter()
                    .copied()
                    .eq(1..normalized.len())
            {
                return Ok(vec![normalized]);
            }
            let retained_policy = attempt
                .retained_object_policy_with_dependencies(normalized.data.normalized_policy());
            Ok(profiles
                .into_iter()
                .map(|profile| {
                    let indices = std::iter::once(profile.material_loop_index)
                        .chain(profile.hole_loop_indices.iter().copied());
                    let boundaries = indices
                        .map(|index| normalized.data.boundary_loops[index].clone())
                        .collect::<Vec<_>>();
                    let loop_count = boundaries.len();
                    let mut data = CurveRegionData2::new(boundaries);
                    data.certified_loop_roles = Some(
                        std::iter::once(CurveRegionLoopRole::Material)
                            .chain(std::iter::repeat_n(
                                CurveRegionLoopRole::Hole,
                                loop_count - 1,
                            ))
                            .collect(),
                    );
                    // Removing other complete material components preserves
                    // this component's regularized boundary. Ownership and
                    // the source normalization remain decision dependencies.
                    data.state = CurveRegionState2::Normalized(retained_policy);
                    data.filled_side_is_left
                        .certify(Arc::from(vec![true; loop_count]));
                    Self {
                        data: Arc::new(data),
                    }
                })
                .collect())
        })
    }

    pub(crate) fn boundary_profiles_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveRegionProfile2<'_>>>> {
        let roles = match self.loop_roles_raw(policy)? {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if roles.len() != self.data.boundary_loops.len() {
            return Err(CurveError::Topology(
                "curve-region role count is inconsistent with boundary loops".into(),
            ));
        }

        let mut profiles = roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| {
                (*role == CurveRegionLoopRole::Material).then_some(CurveRegionProfile2 {
                    material_loop_index: index,
                    material: &self.data.boundary_loops[index],
                    hole_loop_indices: Vec::new(),
                    holes: Vec::new(),
                })
            })
            .collect::<Vec<_>>();
        if profiles.is_empty() {
            return if roles.is_empty() {
                Ok(Classification::Decided(profiles))
            } else {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            };
        }

        for (hole_index, role) in roles.iter().enumerate() {
            if *role != CurveRegionLoopRole::Hole {
                continue;
            }
            let point = match retained_loop_sample_point_evidence(
                &self.data.boundary_loops[hole_index],
                policy,
            )? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };

            let mut owner: Option<usize> = None;
            for (profile_index, profile) in profiles.iter().enumerate() {
                let material_index = profile.material_loop_index;
                let containment = classify_point_evidence_against_retained_loop(
                    self,
                    material_index,
                    &point,
                    policy,
                )?;
                match containment {
                    Classification::Decided(
                        ContourPointLocation::Inside | ContourPointLocation::Boundary,
                    ) => match owner {
                        None => owner = Some(profile_index),
                        Some(owner_index) => {
                            let candidate_point = match retained_loop_sample_point_evidence(
                                profile.material,
                                policy,
                            )? {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                            let current_owner = &profiles[owner_index];
                            let current_material_index = current_owner.material_loop_index;
                            let candidate_inside_owner =
                                classify_point_evidence_against_retained_loop(
                                    self,
                                    current_material_index,
                                    &candidate_point,
                                    policy,
                                )?;
                            match candidate_inside_owner {
                                Classification::Decided(
                                    ContourPointLocation::Inside | ContourPointLocation::Boundary,
                                ) => owner = Some(profile_index),
                                Classification::Decided(ContourPointLocation::Outside) => {
                                    let owner_point = match retained_loop_sample_point_evidence(
                                        current_owner.material,
                                        policy,
                                    )? {
                                        Classification::Decided(point) => point,
                                        Classification::Uncertain(reason) => {
                                            return Ok(Classification::Uncertain(reason));
                                        }
                                    };
                                    let owner_inside_candidate =
                                        classify_point_evidence_against_retained_loop(
                                            self,
                                            material_index,
                                            &owner_point,
                                            policy,
                                        )?;
                                    match owner_inside_candidate {
                                        Classification::Decided(
                                            ContourPointLocation::Inside
                                            | ContourPointLocation::Boundary,
                                        ) => {}
                                        Classification::Decided(ContourPointLocation::Outside) => {
                                            return Ok(Classification::Uncertain(
                                                UncertaintyReason::Ordering,
                                            ));
                                        }
                                        Classification::Uncertain(reason) => {
                                            return Ok(Classification::Uncertain(reason));
                                        }
                                    }
                                }
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                    },
                    Classification::Decided(ContourPointLocation::Outside) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let Some(owner) = owner else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            profiles[owner].hole_loop_indices.push(hole_index);
            profiles[owner]
                .holes
                .push(&self.data.boundary_loops[hole_index]);
        }
        Ok(Classification::Decided(profiles))
    }

    /// Returns the certified internal line/arc accelerator when this region has one.
    pub(crate) fn native_line_arc_region(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<&LineArcRegion2>> {
        let cached = resolve_cached_classification(
            &self.data.line_image_region,
            policy,
            |attempt| -> CurveResult<Classification<Option<LineArcRegion2>>> {
                match self.retained_native_line_arc_region(attempt)? {
                    Classification::Decided(region) => {
                        return Ok(Classification::Decided(Some(region)));
                    }
                    Classification::Uncertain(UncertaintyReason::Unsupported) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                if self.data.certified_loop_roles.is_some() {
                    match self.certified_line_image_region(attempt)? {
                        Classification::Decided(region) => {
                            Ok(Classification::Decided(Some(region)))
                        }
                        Classification::Uncertain(UncertaintyReason::Unsupported) => {
                            Ok(Classification::Decided(None))
                        }
                        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                    }
                } else {
                    match self.line_image_roles_and_contours_raw(attempt)? {
                        Classification::Decided((roles, contours)) => {
                            let region = self.region_from_line_contours(&contours, &roles)?;
                            Ok(Classification::Decided(Some(region)))
                        }
                        Classification::Uncertain(UncertaintyReason::Unsupported) => {
                            Ok(Classification::Decided(None))
                        }
                        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                    }
                }
            },
        )?;
        Ok(match cached {
            Classification::Decided(Some(region)) => Classification::Decided(region),
            Classification::Decided(None) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    /// Borrows an exact line/arc representation when the unified boundary has one.
    ///
    /// This adapter never segments a higher-order carrier and exposes no
    /// independent Boolean, offset, or corner-edit engine.
    pub fn native_contours_fast_path(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<CurveRegionNativeContourView2<'_>>>> {
        resolve_certified_operation(policy, |attempt| {
            self.native_contours_fast_path_raw(attempt)
        })
    }

    pub(crate) fn native_contours_fast_path_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveRegionNativeContourView2<'_>>> {
        self.native_line_arc_region(policy).map(|native| {
            native.map(|native| CurveRegionNativeContourView2 {
                material_contours: native.material_contours(),
                hole_contours: native.hole_contours(),
            })
        })
    }

    /// Solves a boundary-loop chamfer from two exact chord setbacks.
    ///
    /// Native line/arc vertices, retained exact line/circle images, and direct
    /// polynomial or rational Bezier carriers use the same exact interaction
    /// solver as open paths. Authored spline and NURBS boundaries are already
    /// canonical native Bezier spans here, so they take that route without a
    /// second decomposition. Every candidate is regularized with this region's
    /// fill semantics; cuts can merge, split, or remove boundary loops.
    /// Interior algebraic Bezier contacts
    /// retain their selected parameters and exact point images, joined by one
    /// compact algebraic chord instead of falling through to historical
    /// contour machinery. `TrimOrExtend` keeps retained straight and direct
    /// polynomial, rational Bezier, or analytic-parallel endpoints in that same
    /// support authority, with selected contacts in its original parameter
    /// chart. Finite envelopes certify admissibility without replacing those
    /// parameters. Rational extensions stay inside the incident endpoint's
    /// first projective pole; analytic replacements certify source regularity
    /// and the absence of parallel cusps on the actual replacement range.
    pub fn chamfer_loop_vertex_by_setbacks(
        &self,
        loop_index: usize,
        vertex_index: usize,
        previous_setback: Real,
        next_setback: Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveCornerSolutions2<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            self.chamfer_loop_vertex_by_setbacks_raw(
                loop_index,
                vertex_index,
                previous_setback,
                next_setback,
                mode,
                attempt,
            )
        })
    }

    fn chamfer_loop_vertex_by_setbacks_raw(
        &self,
        loop_index: usize,
        vertex_index: usize,
        previous_setback: Real,
        next_setback: Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveCornerSolutions2<Self>> {
        let boundary = self.data.boundary_loops.get(loop_index).ok_or_else(|| {
            curve_region_edit_error(CurveOperation2::Chamfer, CurveError::InvalidCurveRange)
        })?;
        let chain = CurveCornerChain2::new(boundary.fragments(), true);
        let solutions = chain.chamfer_vertex_by_setbacks(
            vertex_index,
            previous_setback,
            next_setback,
            mode,
            policy,
        )?;
        try_map_corner_solutions(solutions, |fragments| {
            self.with_corner_chain_replaced(loop_index, fragments, CurveOperation2::Chamfer, policy)
        })
    }

    /// Solves a boundary-loop circular fillet from an exact radius.
    ///
    /// Exact candidates come from the same carrier-interaction authority used
    /// by open [`CurvePath2`] editing. Edited boundaries are regularized with
    /// this region's fill semantics before publication. Represented direct
    /// and canonical spline/NURBS Bezier trims
    /// use the retained path authority. Retained affine algebraic chords and
    /// direct polynomial or rational Beziers paired with affine lines also
    /// support exact exterior-ray fillet contacts. Direct Bezier incident
    /// cells are partitioned at projective and regularity barriers, and retain
    /// exact or algebraic cuts without endpoint materialization.
    pub fn fillet_loop_vertex(
        &self,
        loop_index: usize,
        vertex_index: usize,
        request: &CurveFillet2,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveCornerSolutions2<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            self.fillet_loop_vertex_raw(loop_index, vertex_index, request, mode, attempt)
        })
    }

    fn fillet_loop_vertex_raw(
        &self,
        loop_index: usize,
        vertex_index: usize,
        request: &CurveFillet2,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveCornerSolutions2<Self>> {
        let boundary = self.data.boundary_loops.get(loop_index).ok_or_else(|| {
            curve_region_edit_error(CurveOperation2::Fillet, CurveError::InvalidCurveRange)
        })?;
        let chain = CurveCornerChain2::new(boundary.fragments(), true);
        let solutions = chain.fillet_vertex(vertex_index, request, mode, policy)?;
        try_map_corner_solutions(solutions, |fragments| {
            self.with_corner_chain_replaced(loop_index, fragments, CurveOperation2::Fillet, policy)
        })
    }

    fn with_corner_chain_replaced(
        &self,
        loop_index: usize,
        fragments: Vec<BezierSplitFragment2>,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        // The corner solver owns the two trim/contact equalities, while the
        // source loop owns every unaffected join. Retained selected fields can
        // make either cut impossible to re-prove by independent Cartesian
        // endpoint comparison, so retain the already-certified chain while
        // normalizing the edited boundaries.
        let edited_loop = CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
            fragments, None, policy,
        )
        .map_err(|cause| curve_region_edit_error(operation, cause))?;

        let roles = match self
            .loop_roles_raw(policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    reason,
                ));
            }
        };
        let fill_rules = self.data.authored_fill_rules().map_or_else(
            || vec![FillRule::EvenOdd; self.data.boundary_loops.len()],
            <[_]>::to_vec,
        );
        let interior_sides = match self
            .filled_side_is_left_raw(policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(sides) => sides
                .iter()
                .map(|left| {
                    if *left {
                        CurveBoundaryInteriorSide2::Left
                    } else {
                        CurveBoundaryInteriorSide2::Right
                    }
                })
                .collect(),
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    reason,
                ));
            }
        };
        let mut loops = self.data.boundary_loops.clone();
        loops[loop_index] = edited_loop;
        let edited = Self::try_new_with_loop_topology(loops, roles, fill_rules, interior_sides)
            .map_err(|cause| curve_region_edit_error(operation, cause))?;
        edited
            .regularized_region_raw(policy)
            .map_err(|error| error.with_operation(operation))
    }

    fn boundary_paths_for_operation(
        &self,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Vec<CurvePath2>>> {
        let mut paths = Vec::with_capacity(self.data.boundary_loops.len());
        for boundary_loop in &self.data.boundary_loops {
            let curves = boundary_loop.curves().to_vec();
            let retained_policy = boundary_loop
                .connectivity_policy
                .or(self.data.normalized_policy())
                .filter(|retained| policy.accepts_retained_policy(*retained));
            let path = if let Some(retained) = retained_policy {
                CurvePath2::from_certified_closed_curves(curves, retained)
            } else if self.data.strict_materialized_connectivity_certified {
                CurvePath2::from_structurally_closed_curves(curves)
            } else {
                CurvePath2::try_new_raw(curves, policy)
                    .map_err(|error| error.with_operation(operation))?
            };
            match crate::curve::validate_closed_curve_path_connectivity(&path, policy)
                .map_err(|error| error.with_operation(operation))?
            {
                Classification::Decided(()) => paths.push(path),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        Ok(Classification::Decided(paths))
    }

    /// Returns every exact boundary as a connected path in traversal order.
    ///
    /// Selected parameters, analytic carriers, and connectivity certificates
    /// survive unchanged. Authored splines retain their promoted exact spans.
    /// No endpoint or curve definition needs scalar materialization.
    pub fn boundary_paths(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<Vec<CurvePath2>>>> {
        resolve_certified_operation(policy, |attempt| {
            self.boundary_paths_for_operation(CurveOperation2::NativeTopology, attempt)
        })
    }

    /// Segments every representable boundary into exact-`Real` line chords.
    ///
    /// Each polynomial or rational span is subdivided until its control hull
    /// certifies the requested source-curve chord-error budget. Material/hole
    /// roles and authored fill rules are preserved in the returned line-only
    /// [`CurveRegion2`]. No coordinate is converted to `f64`; the operation is
    /// nevertheless explicitly lossy with respect to the source curve image.
    /// Use [`Self::project_to_finite_profiles`] for direct mesh/IO output and
    /// [`Self::recover_from_finite_profiles`] for its reconstruction counterpart.
    pub fn segment_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<CurveRegionCertifiedSegmentationResult2>>>
    {
        resolve_certified_operation(policy, |attempt| {
            self.segment_certified_raw(options, attempt)
        })
    }

    fn segment_certified_raw(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<CurveRegionCertifiedSegmentationResult2>> {
        let paths = match self.boundary_paths_for_operation(CurveOperation2::Subdivision, policy)? {
            Classification::Decided(paths) => paths,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let roles = match self
            .loop_roles_raw(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Subdivision, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let fill_rules = self
            .data
            .authored_fill_rules()
            .map_or_else(|| vec![FillRule::EvenOdd; paths.len()], <[_]>::to_vec);
        if paths.len() != roles.len() || paths.len() != fill_rules.len() {
            return Err(curve_region_edit_error(
                CurveOperation2::Subdivision,
                CurveError::Topology(
                    "segmented curve-region semantics do not match boundary loops".into(),
                ),
            ));
        }

        let mut material = Vec::new();
        let mut holes = Vec::new();
        let mut loop_evidence = Vec::with_capacity(paths.len());
        for ((path, role), fill_rule) in paths.iter().zip(roles).zip(fill_rules) {
            let segmented = match path.segment_certified(options, policy)? {
                Classification::Decided(segmented) => segmented,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut segments = Vec::with_capacity(segmented.points().len().saturating_sub(1));
            for edge in segmented.points().windows(2) {
                segments.push(Segment2::Line(
                    LineSeg2::try_new(edge[0].clone(), edge[1].clone()).map_err(|cause| {
                        curve_region_edit_error(CurveOperation2::Subdivision, cause)
                    })?,
                ));
            }
            let contour = Contour2::try_new_with_fill_rule(segments, fill_rule)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Subdivision, cause))?;
            loop_evidence.push(CurveRegionSegmentationLoopEvidence2 {
                role,
                fill_rule,
                source_curve_count: path.curves().len(),
                source_fragment_count: segmented.source_fragment_count(),
                output_segment_count: segmented.certificate().segment_count(),
                max_depth: segmented.certificate().max_depth(),
            });
            match role {
                CurveRegionLoopRole::Material => material.push(contour),
                CurveRegionLoopRole::Hole => holes.push(contour),
            }
        }

        let region = Self::try_from_native_contours_raw(material, holes, policy)?
            .regularized_region_raw(policy)
            .map_err(|error| error.with_operation(CurveOperation2::Subdivision))?;
        Ok(Classification::Decided(
            CurveRegionCertifiedSegmentationResult2 {
                region,
                evidence: CurveRegionCertifiedSegmentationEvidence2 {
                    max_source_chord_error: options.max_error().clone(),
                    loop_evidence,
                    lossy_boundary: true,
                },
            },
        ))
    }

    /// Builds the exact filled stroke of a connected curve path.
    ///
    /// `half_width` must be strictly positive. Each promoted source span is
    /// bounded by its two exact analytic parallels, every authored corner uses
    /// `corner_style`, and open endpoints use `cap_style`. The resulting span,
    /// corner, and cap loops enter the same single signed arrangement as region
    /// offset bands, so self-crossing paths and overlapping stroke bands do not
    /// escape as raw linework or create sequential intermediate regions. Closed
    /// paths receive their cyclic corner and ignore endpoint caps.
    ///
    /// General polynomial, rational, B-spline, and NURBS paths retain analytic
    /// parallel carriers rather than being chordized. `STRICT` accepts only
    /// certified predicates; `APPROXIMATE_512` may consume its terminal
    /// equality policy while returning the same exact carrier geometry.
    pub fn stroke_path(
        path: &CurvePath2,
        half_width: Real,
        corner_style: &OffsetCornerStyle2,
        cap_style: OffsetCap,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        let family = path.curves()[0].family();
        resolve_certified_operation(policy, |attempt| {
            match Self::stroke_path_raw(path, half_width.clone(), corner_style, cap_style, attempt)?
            {
                Classification::Decided(region) => Ok(region),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::Offset,
                    family,
                    reason,
                )),
            }
        })
    }

    fn stroke_path_raw(
        path: &CurvePath2,
        half_width: Real,
        corner_style: &OffsetCornerStyle2,
        cap_style: OffsetCap,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Self>> {
        let family = path.curves()[0].family();
        match crate::curve::validate_curve_path_connectivity(path, policy)
            .map_err(|error| error.with_operation(CurveOperation2::Offset))?
        {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match real_sign(&half_width, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Offset,
                    family,
                    CurveError::InvalidOffsetOptions,
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        match crate::offset::validate_offset_corner_style(corner_style, policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let closed = match crate::curve::curve_path_is_closed(path, policy) {
            Classification::Decided(closed) => closed,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let native = match path
            .native_bezier_fragments_with_policy(policy)
            .map_err(|error| error.with_operation(CurveOperation2::Offset))?
        {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source_fragments = native
            .iter()
            .map(|fragment| BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: fragment.native_curve().clone(),
            })
            .collect::<Vec<_>>();
        let left_spans =
            match exact_offset_span_runs_from_open_path(&source_fragments, &half_width, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(spans) => spans,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let right_distance = -half_width.clone();
        let right_spans =
            match exact_offset_span_runs_from_open_path(&source_fragments, &right_distance, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(spans) => spans,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        if left_spans.len() != right_spans.len() || left_spans.is_empty() {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology(
                    "exact path stroke produced inconsistent source-span inventories".into(),
                ),
            ));
        }

        let join_count = if closed {
            left_spans.len()
        } else {
            left_spans.len().saturating_sub(1)
        };
        let band_capacity = left_spans
            .len()
            .saturating_add(join_count.saturating_mul(2))
            .saturating_add(if closed { 0 } else { 2 });
        let mut band_loops = Vec::with_capacity(band_capacity);
        let mut band_filled_sides = Vec::with_capacity(band_capacity);
        for (right, left) in right_spans.iter().zip(&left_spans) {
            let boundary = match exact_offset_span_band_loop(right, left, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(boundary) => boundary,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            band_loops.push(boundary);
            band_filled_sides.push(true);
        }

        for index in 0..join_count {
            let next = (index + 1) % left_spans.len();
            let reversal = match exact_offset_spans_form_reversal(
                &left_spans[index],
                &left_spans[next],
                policy,
            )
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(reversal) => reversal,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            // Both signed offset sides describe the same join at an exact
            // 180-degree turn. Round owns one oriented semicircular sector
            // from the positive side; bevel and bounded miter reduce to the
            // butt edge already owned by the adjacent span bands.
            if reversal && !matches!(corner_style, OffsetCornerStyle2::Round) {
                continue;
            }
            let sides = [(&left_spans, &half_width), (&right_spans, &right_distance)];
            for (spans, distance) in sides.into_iter().take(if reversal { 1 } else { 2 }) {
                let corner = match exact_offset_corner_band(
                    &spans[index],
                    &spans[next],
                    distance,
                    corner_style,
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                {
                    Classification::Decided(corner) => corner,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if let Some((boundary, filled_side_is_left)) = corner {
                    band_loops.push(boundary);
                    band_filled_sides.push(filled_side_is_left);
                }
            }
        }

        if !closed {
            match cap_style {
                OffsetCap::Butt => {}
                OffsetCap::Round => {
                    for center in [
                        native[0].curve().start(),
                        native.last().expect("nonempty native path").curve().end(),
                    ] {
                        band_loops.push(exact_round_path_cap_band(center, &half_width, policy)?);
                        band_filled_sides.push(true);
                    }
                }
                OffsetCap::Square => {
                    let start_tangent = match exact_path_endpoint_unit_tangent(path, true, policy)?
                    {
                        Classification::Decided(tangent) => tangent,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let end_tangent = match exact_path_endpoint_unit_tangent(path, false, policy)? {
                        Classification::Decided(tangent) => tangent,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let start_dx = &start_tangent.0 * &half_width;
                    let start_dy = &start_tangent.1 * &half_width;
                    let start_extension = LineSeg2::try_new(
                        native[0].curve().start().translated(-start_dx, -start_dy),
                        native[0].curve().start().clone(),
                    )
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
                    let end_extension = LineSeg2::try_new(
                        native
                            .last()
                            .expect("nonempty native path")
                            .curve()
                            .end()
                            .clone(),
                        native
                            .last()
                            .expect("nonempty native path")
                            .curve()
                            .end()
                            .translated(&end_tangent.0 * &half_width, &end_tangent.1 * &half_width),
                    )
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
                    for extension in [start_extension, end_extension] {
                        match exact_line_stroke_band(extension, &half_width, policy)? {
                            Classification::Decided(boundary) => {
                                band_loops.push(boundary);
                                band_filled_sides.push(true);
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                }
            }
        }

        // Span, join, and cap loops are all positive stroke material. One
        // signed arrangement computes their exact union and self-contact
        // regularization without growing a chain of intermediate regions.
        let region =
            regularized_exact_offset_band_arrangement(band_loops, band_filled_sides, policy)?;
        Ok(Classification::Decided(region))
    }

    /// Offsets every boundary so positive distance expands the filled region.
    ///
    /// The exact filled side of each loop selects the required signed left
    /// offset: material exteriors move away from fill while hole boundaries
    /// move into their voids. Independently offset material components and
    /// voids are unioned, then the unified void set is subtracted, so overlaps
    /// created by expansion are returned as regularized boundary topology.
    /// Native line/arc, materialized polynomial, and rational spans lower to exact analytic
    /// parallels, split at every certified offset cusp, and retain exact PH
    /// materializations where available. Round joins are exact circular conics;
    /// bevel and bounded-miter joins are exact lines. Every general result then
    /// passes through the same authoritative exact arrangement that removes
    /// self-walk branches and composes material and hole loops. Certified convex
    /// all-line contractions, non-convex collapses, and higher carriers use the
    /// same boundary-walk/band construction and authoritative arrangement.
    /// Unsupported retained source fragments return explicit uncertainty rather
    /// than sampled geometry. After corner-option validation, an empty region
    /// remains empty for every signed distance without requiring its sign.
    pub fn offset(
        &self,
        distance: Real,
        corner_style: &OffsetCornerStyle2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        crate::policy::resolve_certified_operation(policy, |attempt| {
            match self.offset_raw(distance.clone(), corner_style, attempt)? {
                Classification::Decided(region) => Ok(region),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::Offset,
                    CurveFamily2::Line,
                    reason,
                )),
            }
        })
    }

    fn offset_raw(
        &self,
        distance: Real,
        corner_style: &OffsetCornerStyle2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Self>> {
        match crate::offset::validate_offset_corner_style(corner_style, policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        if matches!(corner_style, OffsetCornerStyle2::Round)
            && !self.data.boundary_loops.is_empty()
            && real_sign(&distance, policy) == Some(RealSign::Negative)
            && self.boundary_fits_strip_narrower_than(&(Real::zero() - &distance), policy)
        {
            // No disk of the erosion radius fits in the filled set, so its
            // round erosion is empty; skip the offset arrangement entirely.
            return Ok(Classification::Decided(Self::empty()));
        }
        // Offsets act on the regularized filled set. Obtain its boundary
        // ownership from the arrangement before asking for incident sides;
        // authored Green integrals are neither necessary nor sufficient for
        // that topology. Already-certified boundaries share their retained
        // data, including for the zero-distance identity.
        self.regularized_region_raw(policy)
            .map_err(|error| error.with_operation(CurveOperation2::Offset))?
            .offset_exact_general_raw(distance, corner_style, policy)
    }

    /// Certifies that the filled set lies between two parallel lines closer
    /// than `2 * radius`, so no disk of `radius` fits inside it.
    ///
    /// Every polynomial Bezier fragment lies in the convex hull of its exact
    /// control points, which therefore contains the filled set. Candidate
    /// strip directions are the fragments' first-to-last control chords; any
    /// direction is sound. With `e` a chord, the hull's extent across `e` is
    /// `(max - min) / |e|` of `cross(e, p)` over control points `p`, compared
    /// squared without roots. Other carriers, unrepresented coordinates or
    /// large boundaries decline.
    fn boundary_fits_strip_narrower_than(&self, radius: &Real, policy: &CurveContext) -> bool {
        const MAX_STRIP_FRAGMENTS: usize = 256;
        let mut points = Vec::new();
        let mut chords = Vec::new();
        for boundary_loop in &self.data.boundary_loops {
            for fragment in boundary_loop.fragments() {
                let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
                    return false;
                };
                let controls: Vec<&Point2> = match curve {
                    BezierSubcurve2::Quadratic(curve) => curve.control_points().to_vec(),
                    BezierSubcurve2::Cubic(curve) => curve.control_points().to_vec(),
                    BezierSubcurve2::RationalQuadratic(_) | BezierSubcurve2::Rational(_) => {
                        return false;
                    }
                };
                chords.push((controls[0].clone(), controls[controls.len() - 1].clone()));
                points.extend(controls.into_iter().cloned());
                if chords.len() > MAX_STRIP_FRAGMENTS {
                    return false;
                }
            }
        }
        if chords.is_empty() {
            return false;
        }
        let four_radius_squared = Real::from(4) * radius * radius;
        chords.iter().any(|(start, end)| {
            let dx = end.x() - start.x();
            let dy = end.y() - start.y();
            let length_squared = &dx * &dx + &dy * &dy;
            if real_sign(&length_squared, policy) != Some(RealSign::Positive) {
                return false;
            }
            let mut extent: Option<(Real, Real)> = None;
            for point in &points {
                let cross = &dx * (point.y() - start.y()) - &dy * (point.x() - start.x());
                extent = Some(match extent {
                    None => (cross.clone(), cross),
                    Some((low, high)) => {
                        let below = compare_reals(&cross, &low, policy);
                        let above = compare_reals(&cross, &high, policy);
                        let (Some(below), Some(above)) = (below, above) else {
                            return false;
                        };
                        (
                            if below.is_lt() { cross.clone() } else { low },
                            if above.is_gt() { cross } else { high },
                        )
                    }
                });
            }
            let Some((low, high)) = extent else {
                return false;
            };
            let width = high - low;
            real_sign(
                &(&width * &width - &four_radius_squared * &length_squared),
                policy,
            ) == Some(RealSign::Negative)
        })
    }

    fn offset_exact_boundary_walk_raw(
        &self,
        distance: &Real,
        corner_style: &OffsetCornerStyle2,
        filled_sides: &[bool],
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Self>> {
        let roles = match self
            .loop_roles_raw(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // The input is a normalized filled boundary. Its authored fill rule
        // has already been consumed. The offset walk can cover an interior
        // face more than once around a fold; parity would incorrectly turn
        // that face into a hole. Retain its nonzero oriented coverage until
        // the arrangement selects the regularized boundary.
        let fill_rules = vec![FillRule::NonZero; self.data.boundary_loops.len()];
        if roles.len() != self.data.boundary_loops.len()
            || fill_rules.len() != self.data.boundary_loops.len()
            || filled_sides.len() != self.data.boundary_loops.len()
        {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology(
                    "exact offset semantics are inconsistent with boundary loops".into(),
                ),
            ));
        }

        let mut certified_convex_filled_left_dilation = self.data.boundary_loops.len() == 1
            && roles[0] == CurveRegionLoopRole::Material
            && filled_sides[0]
            && self.has_regularized_filled_left_topology(policy);
        let mut offset_loops = Vec::with_capacity(self.data.boundary_loops.len());
        for (loop_index, boundary_loop) in self.data.boundary_loops.iter().enumerate() {
            if certified_convex_filled_left_dilation {
                for fragment in boundary_loop.fragments() {
                    if !fragment_certifies_nonnegative_turn(fragment, policy)
                        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                    {
                        certified_convex_filled_left_dilation = false;
                        break;
                    }
                }
            }
            let signed_left_distance = if filled_sides[loop_index] {
                Real::zero() - distance
            } else {
                distance.clone()
            };
            let span_runs = match exact_offset_span_runs_from_boundary_loop(
                boundary_loop,
                &signed_left_distance,
                policy,
            )
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(runs) => runs,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if span_runs.is_empty() {
                return Err(curve_region_edit_error(
                    CurveOperation2::Offset,
                    CurveError::Topology("exact offset loop has no source spans".into()),
                ));
            }
            let spans = span_runs
                .into_iter()
                .map(|(span, _)| span)
                .collect::<Vec<_>>();
            if certified_convex_filled_left_dilation {
                for span_index in 0..spans.len() {
                    let next_index = (span_index + 1) % spans.len();
                    let Some((previous_tangent, next_tangent)) = spans[span_index]
                        .end_tangent
                        .as_ref()
                        .zip(spans[next_index].start_tangent.as_ref())
                    else {
                        certified_convex_filled_left_dilation = false;
                        break;
                    };
                    match curve_tangent_cross_sign(previous_tangent, next_tangent, policy) {
                        Classification::Decided(RealSign::Positive) => {}
                        Classification::Decided(RealSign::Zero) => {
                            if curve_tangents_are_opposite(previous_tangent, next_tangent, policy)
                                != Classification::Decided(false)
                            {
                                certified_convex_filled_left_dilation = false;
                                break;
                            }
                        }
                        Classification::Decided(RealSign::Negative)
                        | Classification::Uncertain(_) => {
                            certified_convex_filled_left_dilation = false;
                            break;
                        }
                    }
                }
            }
            let fragment_capacity = spans
                .iter()
                .map(|span| span.fragments.len())
                .sum::<usize>()
                .saturating_add(spans.len().saturating_mul(2));
            let mut fragments = Vec::with_capacity(fragment_capacity);
            for span_index in 0..spans.len() {
                fragments.extend(spans[span_index].fragments.iter().cloned());
                let next_index = (span_index + 1) % spans.len();
                match append_exact_offset_join(
                    &mut fragments,
                    &spans[span_index],
                    &spans[next_index],
                    &signed_left_distance,
                    corner_style,
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                {
                    Classification::Decided(corner) => corner,
                    Classification::Uncertain(reason) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-exact-offset-blocker",
                            "join",
                        );
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let arrangement_sources = certified_convex_filled_left_dilation.then(|| {
                (0..fragments.len())
                    .map(|fragment_index| {
                        CurveRegionFragmentSource2::new(fragment_index, fragment_index, 0)
                    })
                    .collect()
            });
            offset_loops.push(
                CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                    fragments,
                    arrangement_sources,
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?,
            );
        }

        let mut raw = Self::new(offset_loops)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
        {
            let data = raw.data_mut_for_construction();
            data.certified_loop_roles = Some(Arc::from(roles));
            data.state = CurveRegionState2::Authored(Arc::from(fill_rules));
        }
        raw = raw
            .with_certified_filled_side_is_left(filled_sides.to_vec())
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
        if certified_convex_filled_left_dilation {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-regularization",
                "convex-boundary-certificate",
            );
            raw = raw
                .with_regularized_filled_left_topology(policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
            let data = raw.data_mut_for_construction();
            data.certified_loop_roles = Some(shared_all_material_curve_region_loop_roles(
                data.boundary_loops.len(),
            ));
            return Ok(Classification::Decided(raw));
        }
        let regularized = raw.regularized_region_raw(policy);
        #[cfg(feature = "dispatch-trace")]
        if regularized.is_err() {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-blocker",
                "regularization",
            );
        }
        regularized
            .map(Classification::Decided)
            .map_err(|error| error.with_operation(CurveOperation2::Offset))
    }

    fn offset_exact_general_raw(
        &self,
        distance: Real,
        corner_style: &OffsetCornerStyle2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Self>> {
        if self.is_empty() || is_zero(&distance, policy) == Some(true) {
            return Ok(Classification::Decided(self.clone()));
        }
        let distance_positive = match real_sign(&distance, policy) {
            Some(RealSign::Positive) => true,
            Some(RealSign::Negative) => false,
            Some(RealSign::Zero) => return Ok(Classification::Decided(self.clone())),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let filled_sides = match self
            .filled_side_is_left_raw(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(sides) => sides,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if filled_sides.len() != self.data.boundary_loops.len() {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology(
                    "exact offset filled-side semantics are inconsistent with boundary loops"
                        .into(),
                ),
            ));
        }
        if distance_positive {
            // Dilation is represented most compactly by its complete offset
            // boundary walk. The authoritative unary arrangement regularizes
            // concave self-contacts and composes holes in one pass. Boundary
            // bands remain the contraction path because they encode medial
            // collapse as exact set subtraction instead of raw winding.
            return self.offset_exact_boundary_walk_raw(
                &distance,
                corner_style,
                filled_sides,
                policy,
            );
        }

        // Union the exact span and corner bands first, then apply the complete
        // band once. This prevents overlapping strips from becoming signed
        // multiplicity while retaining one authoritative Boolean application
        // for post-collapse neck removal.
        let mut band_loops = Vec::new();
        let mut band_filled_sides = Vec::new();
        for (loop_index, boundary_loop) in self.data.boundary_loops.iter().enumerate() {
            let signed_left_distance = if filled_sides[loop_index] {
                Real::zero() - &distance
            } else {
                distance.clone()
            };
            let band_filled_side_is_left = match real_sign(&signed_left_distance, policy) {
                Some(RealSign::Positive) => true,
                Some(RealSign::Negative) => false,
                Some(RealSign::Zero) => unreachable!("zero offset returned before band assembly"),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            let opposite_distance = -signed_left_distance.clone();
            let span_runs = match exact_offset_span_runs_from_boundary_loop(
                boundary_loop,
                &signed_left_distance,
                policy,
            )
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(runs) => runs,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let opposite_runs = match exact_offset_span_runs_from_boundary_loop(
                boundary_loop,
                &opposite_distance,
                policy,
            )
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(runs) => runs,
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-blocker",
                        "opposite-band-span",
                    );
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if span_runs.len() != opposite_runs.len()
                || span_runs
                    .iter()
                    .zip(&opposite_runs)
                    .any(|((_, consumed), (_, opposite_consumed))| consumed != opposite_consumed)
            {
                return Err(curve_region_edit_error(
                    CurveOperation2::Offset,
                    CurveError::Topology(
                        "opposite exact offset bands coalesced different source runs".into(),
                    ),
                ));
            }
            let spans = span_runs
                .into_iter()
                .map(|(span, _)| span)
                .collect::<Vec<_>>();
            let opposite_spans = opposite_runs
                .into_iter()
                .map(|(span, _)| span)
                .collect::<Vec<_>>();
            if spans.is_empty() {
                return Err(curve_region_edit_error(
                    CurveOperation2::Offset,
                    CurveError::Topology("exact offset loop has no source spans".into()),
                ));
            }

            for span_index in 0..spans.len() {
                let band = match exact_offset_span_band_loop(
                    &opposite_spans[span_index],
                    &spans[span_index],
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                {
                    Classification::Decided(band) => band,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                band_loops.push(band);
                band_filled_sides.push(band_filled_side_is_left);
            }

            for span_index in 0..spans.len() {
                let next_index = (span_index + 1) % spans.len();
                let corner = match exact_offset_corner_band(
                    &spans[span_index],
                    &spans[next_index],
                    &signed_left_distance,
                    corner_style,
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                {
                    Classification::Decided(corner) => corner,
                    Classification::Uncertain(reason) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-exact-offset-blocker",
                            "join",
                        );
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                // Adjacent span bands already own inner corners. Add only the
                // exact outward sector selected by the shared corner solver.
                let Some((band, corner_filled_side_is_left)) = corner else {
                    continue;
                };
                band_loops.push(band);
                band_filled_sides.push(corner_filled_side_is_left);
            }
        }

        if band_loops.is_empty() {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology("exact offset produced no boundary bands".into()),
            ));
        }
        let bands =
            regularized_exact_offset_band_arrangement(band_loops, band_filled_sides, policy)?;
        let regularized = self.boolean_region_raw(
            &bands,
            if distance_positive {
                BooleanOp::Union
            } else {
                BooleanOp::Difference
            },
            policy,
        );
        #[cfg(feature = "dispatch-trace")]
        if regularized.is_err() {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-blocker",
                "band-application",
            );
        }
        regularized
            .map(Classification::Decided)
            .map_err(|error| error.with_operation(CurveOperation2::Offset))
    }

    fn certified_line_image_region(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<LineArcRegion2>> {
        let Some(roles) = self.data.certified_loop_roles.as_deref() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let mut contours = Vec::with_capacity(self.data.boundary_loops.len());
        for boundary_loop in &self.data.boundary_loops {
            match retained_line_loop_to_contour(boundary_loop, policy)? {
                Classification::Decided(contour) => contours.push(contour),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        self.region_from_line_contours(&contours, roles)
            .map(Classification::Decided)
    }

    fn retained_native_line_arc_region(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<LineArcRegion2>> {
        let mut contours = Vec::with_capacity(self.data.boundary_loops.len());
        for (loop_index, boundary_loop) in self.data.boundary_loops.iter().enumerate() {
            let fill_rule = self
                .data
                .authored_fill_rules()
                .and_then(|rules| rules.get(loop_index))
                .copied()
                .unwrap_or(FillRule::NonZero);
            match retained_native_loop_to_contour(boundary_loop, fill_rule, policy)? {
                Classification::Decided(contour) => contours.push(contour),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let roles = match self.loop_roles_raw(policy)? {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.region_from_line_contours(&contours, &roles)
            .map(Classification::Decided)
    }

    fn region_from_line_contours(
        &self,
        contours: &[Contour2],
        roles: &[CurveRegionLoopRole],
    ) -> CurveResult<LineArcRegion2> {
        if roles.len() != contours.len() {
            return Err(CurveError::Topology(
                "curve-region certified role count is inconsistent with line contours".into(),
            ));
        }
        if self
            .data
            .authored_fill_rules()
            .is_some_and(|rules| rules.len() != contours.len())
        {
            return Err(CurveError::Topology(
                "curve-region fill-rule count is inconsistent with line contours".into(),
            ));
        }

        let mut material = Vec::new();
        let mut holes = Vec::new();
        for (index, (contour, role)) in contours.iter().zip(roles).enumerate() {
            let contour = match self.data.authored_fill_rules() {
                Some(fill_rules) if contour.fill_rule() != fill_rules[index] => {
                    Contour2::try_new_with_fill_rule(
                        contour.segments().to_vec(),
                        fill_rules[index],
                    )?
                }
                _ => contour.clone(),
            };
            match role {
                CurveRegionLoopRole::Material => material.push(contour),
                CurveRegionLoopRole::Hole => holes.push(contour),
            }
        }
        Ok(LineArcRegion2::new(material, holes))
    }

    /// Classifies an exact point against the retained region.
    ///
    /// Scalar coordinates and selected algebraic fields keep their specialized
    /// predicates. Generated points reuse their retained evidence and the same
    /// boundary ownership as Boolean operations, without materializing coordinates.
    pub fn classify_point(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<RegionPointLocation>>> {
        resolve_certified_operation(policy, |attempt| {
            self.classify_curve_point_raw(point, attempt, &mut None)
        })
    }

    /// Classifies a batch of exact points against the unified region.
    ///
    /// Native query indexes and generated-point boundary preparation are shared
    /// within the batch. Input order, selected evidence and the operation-wide
    /// policy terminal are preserved, including in mixed-representation batches.
    pub fn classify_points(
        &self,
        points: &[CurvePoint2],
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Vec<Classification<RegionPointLocation>>>> {
        resolve_certified_operation(policy, |attempt| {
            let native = if points.iter().any(|point| point.coordinates().is_some())
                && self.has_regularized_filled_left_topology(attempt)
                && let Classification::Decided(native) = self.native_line_arc_region(attempt)?
            {
                Some(crate::prepared::RegionQuery2::from_region_view(
                    &native.as_view(),
                    attempt,
                ))
            } else {
                None
            };
            let mut prepared = None;
            points
                .iter()
                .map(|point| {
                    if let (Some(native), Some(coordinates)) = (&native, point.coordinates())
                        && let decided @ Classification::Decided(_) =
                            native.classify_point(coordinates, attempt)
                    {
                        return Ok(decided);
                    }
                    self.classify_curve_point_raw(point, attempt, &mut prepared)
                })
                .collect()
        })
    }

    fn classify_curve_point_raw<'a>(
        &'a self,
        point: &CurvePoint2,
        policy: &CurveContext,
        prepared: &mut Option<crate::curve_region_boolean::CurveRegionBooleanContext<'a>>,
    ) -> CurveResult<Classification<RegionPointLocation>> {
        match point {
            CurvePoint2(CurvePointData2::Exact(point)) => self.classify_point_raw(point, policy),
            CurvePoint2(CurvePointData2::Algebraic(point)) => {
                self.classify_algebraic_point_raw(point, policy)
            }
            _ => crate::curve_region_boolean::classify_retained_point_evidence_against_region_by_probe(
                self, point.clone(), policy, prepared,
            ),
        }
    }

    /// Returns native line/arc structural facts when that exact specialization exists.
    ///
    /// Higher-order retained carriers remain explicitly unsupported because
    /// [`crate::RegionFacts`] describes native segment-family facts and must not
    /// silently flatten a curved boundary.
    pub fn structural_facts(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<crate::RegionFacts>>> {
        resolve_certified_operation(policy, |attempt| {
            Ok(match self.native_line_arc_region(attempt)? {
                Classification::Decided(native) => {
                    Classification::Decided(native.structural_facts())
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            })
        })
    }

    pub(crate) fn classify_algebraic_point_raw(
        &self,
        point: &RationalBezierAlgebraicPointImage2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RegionPointLocation>> {
        self.classify_algebraic_point_with_boundary_contract(point, policy, true)
    }

    pub(crate) fn classify_algebraic_point_off_boundary_raw(
        &self,
        point: &RationalBezierAlgebraicPointImage2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RegionPointLocation>> {
        self.classify_algebraic_point_with_boundary_contract(point, policy, false)
    }

    fn classify_algebraic_point_with_boundary_contract(
        &self,
        point: &RationalBezierAlgebraicPointImage2,
        policy: &CurveContext,
        certify_boundary: bool,
    ) -> CurveResult<Classification<RegionPointLocation>> {
        if self
            .data
            .certified_loop_roles
            .as_ref()
            .is_some_and(|roles| roles.len() != self.data.boundary_loops.len())
            || self
                .data
                .authored_fill_rules()
                .is_some_and(|rules| rules.len() != self.data.boundary_loops.len())
        {
            return Err(CurveError::Topology(
                "curve-region loop semantics are inconsistent with boundary loops".into(),
            ));
        }
        if self.data.boundary_loops.is_empty() {
            return Ok(Classification::Decided(RegionPointLocation::Outside));
        }
        let predicates = match point.predicate_evaluator(policy)? {
            Classification::Decided(predicates) => predicates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut inside = false;
        let mut signed_depth = 0_i32;
        for (loop_index, boundary_loop) in self.data.boundary_loops.iter().enumerate() {
            if let Classification::Decided(bounds) =
                retained_loop_query_bounds(boundary_loop, policy)
                && algebraic_point_is_decided_outside_bounds(&predicates, &bounds, policy)?
            {
                continue;
            }
            let fill_rule = self
                .data
                .authored_fill_rules()
                .map_or(FillRule::EvenOdd, |rules| rules[loop_index]);
            match classify_algebraic_point_against_retained_loop(
                boundary_loop,
                &predicates,
                fill_rule,
                certify_boundary,
                policy,
            )? {
                Classification::Decided(ContourPointLocation::Inside) => {
                    if let Some(roles) = &self.data.certified_loop_roles {
                        signed_depth += match roles[loop_index] {
                            CurveRegionLoopRole::Material => 1,
                            CurveRegionLoopRole::Hole => -1,
                        };
                    } else {
                        inside = !inside;
                    }
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(Classification::Decided(RegionPointLocation::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let inside = self
            .data
            .certified_loop_roles
            .as_ref()
            .map_or(inside, |_| signed_depth > 0);
        Ok(Classification::Decided(if inside {
            RegionPointLocation::Inside
        } else {
            RegionPointLocation::Outside
        }))
    }

    pub(crate) fn classify_point_raw(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RegionPointLocation>> {
        if self.has_regularized_filled_left_topology(policy) {
            match self.native_line_arc_region(policy)? {
                Classification::Decided(region) => {
                    let classification = region.classify_point(point, policy);
                    if matches!(classification, Classification::Decided(_)) {
                        return Ok(classification);
                    }
                    // Native line/arc lowering is only a fast path.  Its
                    // coordinate-form arc predicates can remain undecided
                    // when retained rational-conic provenance still supplies
                    // a direct exact polynomial winding certificate.
                    return classify_point_against_retained_loops(
                        &self.data.boundary_loops,
                        point,
                        policy,
                        self.data.certified_loop_roles.as_deref(),
                        self.data.authored_fill_rules(),
                    );
                }
                // Native lowering is only a specialization. Any undecided
                // construction predicate, not just an unsupported carrier,
                // must fall through to the authoritative retained winding
                // kernel below.
                Classification::Uncertain(_) => {}
            }
        }
        let Some(native_loops) = self.native_boundary_loops() else {
            return classify_point_against_retained_loops(
                &self.data.boundary_loops,
                point,
                policy,
                self.data.certified_loop_roles.as_deref(),
                self.data.authored_fill_rules(),
            );
        };
        if self
            .data
            .certified_loop_roles
            .as_ref()
            .is_some_and(|roles| roles.len() != native_loops.len())
            || self
                .data
                .authored_fill_rules()
                .is_some_and(|rules| rules.len() != native_loops.len())
        {
            return Err(CurveError::Topology(
                "curve-region loop semantics are inconsistent with native boundary loops".into(),
            ));
        }
        let native_bounds = self.native_boundary_bounds(policy);
        let mut inside = false;
        let mut signed_depth = 0_i32;
        for (index, boundary_loop) in native_loops.iter().enumerate() {
            if native_bounds.is_some_and(|bounds| {
                matches!(
                    bounds[index].contains_point(point, policy),
                    Classification::Decided(false)
                )
            }) {
                continue;
            }
            let fill_rule = self
                .data
                .authored_fill_rules()
                .map_or(FillRule::EvenOdd, |rules| rules[index]);
            match classify_point_against_native_loop_after_bounds_with_fill_rule(
                boundary_loop,
                point,
                fill_rule,
                policy,
            )? {
                Classification::Decided(ContourPointLocation::Inside) => {
                    if let Some(roles) = &self.data.certified_loop_roles {
                        signed_depth += match roles[index] {
                            CurveRegionLoopRole::Material => 1,
                            CurveRegionLoopRole::Hole => -1,
                        };
                    } else {
                        inside = !inside;
                    }
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(Classification::Decided(RegionPointLocation::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let inside = self
            .data
            .certified_loop_roles
            .as_ref()
            .map_or(inside, |_| signed_depth > 0);
        Ok(Classification::Decided(if inside {
            RegionPointLocation::Inside
        } else {
            RegionPointLocation::Outside
        }))
    }

    pub(crate) fn loop_windings_from_boundary_side_ray(
        &self,
        point: &Point2,
        direction_x: Real,
        direction_y: Real,
        direction_is_certified_nonzero: bool,
        source_crossing_direction: BezierLineCrossingDirection,
        source_loop_index: usize,
        source_fragment_index: usize,
        source_parameter: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<i32>>> {
        let direction_squared = &direction_x * &direction_x + &direction_y * &direction_y;
        match real_sign(&direction_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "boundary-side ray direction has a negative squared norm".into(),
                ));
            }
            None if direction_is_certified_nonzero => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        if source_loop_index >= self.data.boundary_loops.len()
            || source_fragment_index
                >= self.data.boundary_loops[source_loop_index]
                    .fragments()
                    .len()
        {
            return Err(CurveError::Topology(
                "boundary-side ray source is outside the retained region".into(),
            ));
        }

        let endpoint = Point2::new(point.x() + &direction_x, point.y() + &direction_y);
        let ray = BezierRay2 {
            line: LineSeg2::try_new(point.clone(), endpoint)?,
            direction_x,
            direction_y,
        };
        let source_tangent_contacts = retained_circle_tangent_contacts(
            &self.data.boundary_loops[source_loop_index].fragments()[source_fragment_index],
        );
        let mut windings = Vec::with_capacity(self.data.boundary_loops.len());
        for (loop_index, boundary_loop) in self.data.boundary_loops.iter().enumerate() {
            let skipped_origin = Some(RetainedRayOriginContact {
                fragment_index: (loop_index == source_loop_index).then_some(source_fragment_index),
                parameter: source_parameter,
                crossing_direction: source_crossing_direction,
                tangent_contacts: source_tangent_contacts,
            });
            match classify_point_with_retained_ray_skipping_origin(
                boundary_loop,
                point,
                &ray,
                skipped_origin,
                policy,
            )? {
                Classification::Decided(RetainedRayWinding::Winding(winding)) => {
                    windings.push(winding);
                }
                Classification::Decided(RetainedRayWinding::Boundary) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(windings))
    }

    pub(crate) fn algebraic_loop_windings_from_boundary_side_ray(
        &self,
        point: &RationalBezierAlgebraicPointImage2,
        direction_x: Real,
        direction_y: Real,
        source_loop_index: usize,
        source_fragment_index: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<i32>>> {
        let direction_squared = &direction_x * &direction_x + &direction_y * &direction_y;
        match real_sign(&direction_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "algebraic boundary-side ray has a negative squared norm".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let Some(source_loop) = self.data.boundary_loops.get(source_loop_index) else {
            return Err(CurveError::Topology(
                "algebraic boundary-side ray source loop is missing".into(),
            ));
        };
        let Some(
            BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_),
        ) = source_loop.fragments().get(source_fragment_index)
        else {
            return Err(CurveError::Topology(
                "algebraic boundary-side ray source is not a retained algebraic fragment".into(),
            ));
        };
        let point = match point.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let side_x = -direction_y.clone();
        let side_y = direction_x.clone();
        let mut windings = Vec::with_capacity(self.data.boundary_loops.len());
        for (loop_index, boundary_loop) in self.data.boundary_loops.iter().enumerate() {
            let fragments = match prepare_algebraic_ray_retained_fragments(boundary_loop, policy)? {
                Classification::Decided(fragments) => fragments,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match algebraic_ray_retained_fragments_admit_direction(
                &fragments, &point, &side_x, &side_y, policy,
            )? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let winding = match algebraic_ray_retained_fragments_winding(
                &fragments,
                &point,
                &direction_x,
                &direction_y,
                (loop_index == source_loop_index).then_some(source_fragment_index),
                true,
                policy,
            )? {
                Classification::Decided(winding) => winding,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            windings.push(winding);
        }
        Ok(Classification::Decided(windings))
    }

    pub(crate) fn region_location_from_loop_windings(
        &self,
        windings: &[i32],
    ) -> CurveResult<RegionPointLocation> {
        self.region_location_from_loop_winding_entries(
            windings.len(),
            windings
                .iter()
                .copied()
                .enumerate()
                .filter(|(_, winding)| *winding != 0),
        )
    }

    /// Classifies from `(loop, winding)` entries; omitted loops have zero
    /// winding, which is outside under every loop fill rule.
    pub(crate) fn region_location_from_loop_winding_entries(
        &self,
        loop_count: usize,
        entries: impl IntoIterator<Item = (usize, i32)>,
    ) -> CurveResult<RegionPointLocation> {
        let windings_len = loop_count;
        if windings_len != self.data.boundary_loops.len()
            || self
                .data
                .certified_loop_roles
                .as_ref()
                .is_some_and(|roles| roles.len() != windings_len)
            || self
                .data
                .authored_fill_rules()
                .is_some_and(|rules| rules.len() != windings_len)
        {
            return Err(CurveError::Topology(
                "curve-region winding vector is inconsistent with boundary loops".into(),
            ));
        }
        let mut inside = false;
        let mut signed_depth = 0_i32;
        for (loop_index, winding) in entries {
            let fill_rule = self
                .data
                .authored_fill_rules()
                .map_or(FillRule::EvenOdd, |rules| rules[loop_index]);
            if winding_location(winding, fill_rule) != ContourPointLocation::Inside {
                continue;
            }
            if let Some(roles) = &self.data.certified_loop_roles {
                signed_depth += match roles[loop_index] {
                    CurveRegionLoopRole::Material => 1,
                    CurveRegionLoopRole::Hole => -1,
                };
            } else {
                inside = !inside;
            }
        }
        let inside = self
            .data
            .certified_loop_roles
            .as_ref()
            .map_or(inside, |_| signed_depth > 0);
        Ok(if inside {
            RegionPointLocation::Inside
        } else {
            RegionPointLocation::Outside
        })
    }

    /// Returns retained boundary loops.
    pub fn boundary_loops(&self) -> &[CurveRegionBoundaryLoop2] {
        &self.data.boundary_loops
    }

    /// Consumes the region and returns retained boundary loops.
    pub fn into_boundary_loops(self) -> Vec<CurveRegionBoundaryLoop2> {
        match Arc::try_unwrap(self.data) {
            Ok(data) => data.boundary_loops,
            Err(data) => data.boundary_loops.clone(),
        }
    }

    /// Returns true when the region has no boundary loops.
    pub fn is_empty(&self) -> bool {
        self.data.boundary_loops.is_empty()
    }

    /// Returns the number of retained boundary loops.
    pub fn len(&self) -> usize {
        self.data.boundary_loops.len()
    }

    /// Returns true when any boundary loop retains non-native algebraic geometry.
    pub fn has_algebraic_fragments(&self) -> bool {
        self.data
            .boundary_loops
            .iter()
            .any(CurveRegionBoundaryLoop2::has_algebraic_fragments)
    }

    /// Returns exact signed area only when all retained loops have implemented
    /// Green integrals or a policy-certified line image.
    pub fn signed_area(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Option<Real>>>> {
        if let Some(area) = self.data.signed_area_cache.certified() {
            return Ok(CurveOutcome::new(
                Classification::Decided(area.clone()),
                CurveCertainty::Certified,
            ));
        }
        resolve_certified_operation(policy, |attempt| self.signed_area_raw(attempt))
    }

    pub(crate) fn signed_area_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        resolve_cached_evaluation(&self.data.signed_area_cache, policy, |attempt| {
            self.compute_signed_area(attempt)
        })
        .map(|classification| classification.map(Clone::clone))
    }

    /// Returns exact material-minus-hole area when every loop has an implemented integral.
    ///
    /// Unlike [`Self::signed_area`], this query uses explicit/nesting-derived
    /// loop roles and ignores authored orientation. Nested material islands add
    /// area while owned holes subtract it.
    /// Per-loop fill rules are applied to repeated windings before role
    /// accumulation. A retained algebraic or otherwise unsupported integral
    /// returns `Decided(None)` rather than approximating the boundary. If exact
    /// self-contact analysis cannot certify a non-repeated loop as simple, the
    /// query remains explicitly uncertain instead of treating traversal
    /// multiplicity as filled-set area.
    pub fn filled_area(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Option<Real>>>> {
        resolve_certified_operation(policy, |attempt| self.filled_area_raw(attempt))
    }

    pub(crate) fn filled_area_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        let mut magnitudes = Vec::with_capacity(self.data.boundary_loops.len());
        if self
            .data
            .authored_fill_rules()
            .is_some_and(|rules| rules.len() != self.data.boundary_loops.len())
        {
            return Err(CurveError::Topology(
                "curve-region filled-area fill rules are inconsistent with boundary loops".into(),
            ));
        }
        for (index, boundary_loop) in self.data.boundary_loops.iter().enumerate() {
            let area = match boundary_loop.signed_area_raw(policy)? {
                Classification::Decided(Some(area)) => area,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let fill_rule = self
                .data
                .authored_fill_rules()
                .map_or(FillRule::EvenOdd, |rules| rules[index]);
            let magnitude = match if self.has_regularized_filled_left_topology(policy) {
                absolute_nonzero_area(area, policy).map(|area| area.map(Some))?
            } else {
                curve_region_loop_filled_area_magnitude(boundary_loop, area, fill_rule, policy)?
            } {
                Classification::Decided(Some(magnitude)) => magnitude,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            magnitudes.push(magnitude);
        }
        let roles = match self.loop_roles_raw(policy)? {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if roles.len() != magnitudes.len() {
            return Err(CurveError::Topology(
                "curve-region filled-area role count is inconsistent with boundary loops".into(),
            ));
        }
        let total =
            roles
                .into_iter()
                .zip(magnitudes)
                .fold(Real::zero(), |total, (role, magnitude)| match role {
                    CurveRegionLoopRole::Material => &total + &magnitude,
                    CurveRegionLoopRole::Hole => &total - &magnitude,
                });
        Ok(Classification::Decided(Some(total)))
    }

    fn compute_signed_area(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        let mut total = Real::zero();
        for boundary_loop in &self.data.boundary_loops {
            match boundary_loop.signed_area_raw(policy)? {
                Classification::Decided(Some(area)) => {
                    total = &total + &area;
                }
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        Ok(Classification::Decided(Some(total)))
    }

    fn native_boundary_loops(&self) -> Option<&[BezierBoundaryLoop2]> {
        self.data
            .native_boundary_loops
            .get_or_init(|| {
                self.data
                    .boundary_loops
                    .iter()
                    .map(retained_loop_to_native)
                    .collect::<Option<Vec<_>>>()
                    .map(Arc::from)
            })
            .as_deref()
    }

    fn native_boundary_bounds(&self, policy: &CurveContext) -> Option<&[Aabb2]> {
        let native_loops = self.native_boundary_loops()?;
        let bounds =
            resolve_cached_classification(&self.data.native_boundary_bounds, policy, |attempt| {
                let mut bounds = Vec::with_capacity(native_loops.len());
                for boundary_loop in native_loops {
                    match native_loop_bounds(boundary_loop, attempt) {
                        Classification::Decided(boundary_bounds) => bounds.push(boundary_bounds),
                        Classification::Uncertain(reason) => {
                            return Ok::<_, core::convert::Infallible>(Classification::Uncertain(
                                reason,
                            ));
                        }
                    }
                }
                Ok(Classification::Decided(Arc::from(bounds)))
            })
            .expect("native boundary bound construction is infallible");
        match bounds {
            Classification::Decided(bounds) => Some(bounds),
            Classification::Uncertain(_) => None,
        }
    }
}

fn curve_region_loop_filled_area_magnitude(
    boundary_loop: &CurveRegionBoundaryLoop2,
    signed_area: Real,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Real>>> {
    if let Some(period) = repeated_boundary_fragment_period(boundary_loop.fragments()) {
        let base_loop =
            CurveRegionBoundaryLoop2::new(boundary_loop.fragments()[..period].to_vec(), policy)?;
        match represented_boundary_loop_is_simple(&base_loop, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let repeat_count = boundary_loop.fragments().len() / period;
        if fill_rule == FillRule::EvenOdd && repeat_count.is_multiple_of(2) {
            return Ok(Classification::Decided(Some(Real::zero())));
        }
        let base_area = match base_loop.signed_area_raw(policy)? {
            Classification::Decided(Some(area)) => area,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        return absolute_nonzero_area(base_area, policy).map(|area| area.map(Some));
    }

    match represented_boundary_loop_is_simple(boundary_loop, policy)? {
        Classification::Decided(true) => {
            absolute_nonzero_area(signed_area, policy).map(|area| area.map(Some))
        }
        Classification::Decided(false) => Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn repeated_boundary_fragment_period(fragments: &[BezierSplitFragment2]) -> Option<usize> {
    let len = fragments.len();
    (1..=len / 2).find(|period| {
        len.is_multiple_of(*period)
            && fragments
                .iter()
                .enumerate()
                .all(|(index, fragment)| fragment == &fragments[index % period])
    })
}

fn absolute_nonzero_area(area: Real, policy: &CurveContext) -> CurveResult<Classification<Real>> {
    Ok(match real_sign(&area, policy) {
        Some(RealSign::Negative) => Classification::Decided(Real::zero() - area),
        Some(RealSign::Positive) => Classification::Decided(area),
        Some(RealSign::Zero) => Classification::Uncertain(UncertaintyReason::Boundary),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    })
}

fn represented_boundary_loop_is_simple(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let mut curves = Vec::with_capacity(boundary_loop.fragments().len());
    for fragment in boundary_loop.fragments() {
        match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                match curve.certified_injective_image(policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                curves.push(Curve2::from(curve.clone()));
            }
            _ => match retained_line_fragment_segment(fragment, policy)? {
                Classification::Decided(line) => curves.push(Curve2::from(line)),
                Classification::Uncertain(_) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
            },
        }
    }
    let path = match CurvePath2::try_new_raw(curves, policy) {
        Ok(path) => path,
        Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
        Err(ExactCurveError::Blocked(blocker)) => {
            return Ok(Classification::Uncertain(blocker.reason()));
        }
    };
    let evidence = match path.intersect_path_raw(&path, policy) {
        Ok(evidence) => evidence,
        Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
        Err(ExactCurveError::Blocked(blocker)) => {
            return Ok(Classification::Uncertain(blocker.reason()));
        }
    };

    if let Some(blocker) = evidence
        .blockers()
        .iter()
        .find(|blocker| blocker.first_curve_index() < blocker.second_curve_index())
    {
        let reason = match blocker.blocker().kind() {
            CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
            CurveIntersectionPairBlockerKind2::IncompleteReplay => UncertaintyReason::Predicate,
            CurveIntersectionPairBlockerKind2::SharedComponent => UncertaintyReason::Boundary,
        };
        return Ok(Classification::Uncertain(reason));
    }
    if evidence
        .overlaps()
        .iter()
        .any(|overlap| overlap.first_curve_index() < overlap.second_curve_index())
    {
        return Ok(Classification::Decided(false));
    }
    for contact in evidence
        .contacts()
        .iter()
        .filter(|contact| contact.first_curve_index() < contact.second_curve_index())
    {
        match curve_path_contact_is_ordinary_adjacent_endpoint(&path, contact, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(false)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(true))
}

fn curve_path_contact_is_ordinary_adjacent_endpoint(
    path: &CurvePath2,
    contact: &CurvePathIntersectionContact2,
    policy: &CurveContext,
) -> Classification<bool> {
    let first_index = contact.first_curve_index();
    let second_index = contact.second_curve_index();
    let consecutive = (second_index == first_index + 1).then(|| {
        (
            path.curves()[first_index].parameter_domain().end(),
            path.curves()[second_index].parameter_domain().start(),
        )
    });
    let closing = (first_index == 0 && second_index + 1 == path.curves().len()).then(|| {
        (
            path.curves()[first_index].parameter_domain().start(),
            path.curves()[second_index].parameter_domain().end(),
        )
    });
    if consecutive.is_none() && closing.is_none() {
        return Classification::Decided(false);
    }
    let (Ok(Classification::Decided(first)), Ok(Classification::Decided(second))) = (
        contact.contact().first().parameter(policy),
        contact.contact().second().parameter(policy),
    ) else {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    };
    let mut uncertain = false;
    for (expected_first, expected_second) in consecutive.into_iter().chain(closing) {
        match (
            first
                .cmp_by_refinement(expected_first, policy)
                .ok()
                .and_then(|r| match r {
                    Classification::Decided(r) => Some(r),
                    _ => None,
                }),
            second
                .cmp_by_refinement(expected_second, policy)
                .ok()
                .and_then(|r| match r {
                    Classification::Decided(r) => Some(r),
                    _ => None,
                }),
        ) {
            (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => {
                return Classification::Decided(true);
            }
            (Some(_), Some(_)) => {}
            _ => uncertain = true,
        }
    }
    if uncertain {
        Classification::Uncertain(UncertaintyReason::Ordering)
    } else {
        Classification::Decided(false)
    }
}

fn affine_region_error(cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(
        CurveOperation2::Transformation,
        CurveFamily2::RationalBezier,
        cause,
    )
}

pub(crate) fn transform_curve_fragment_similarity(
    fragment: &BezierSplitFragment2,
    transform: &crate::Similarity2,
    policy: &CurveContext,
) -> ExactCurveResult<BezierSplitFragment2> {
    let (m00, m01, m10, m11, tx, ty) = transform.affine_components();
    let similarity = std::cell::OnceCell::from(Some(transform.clone()));
    transform_retained_region_fragment(
        fragment,
        m00,
        m01,
        m10,
        m11,
        tx,
        ty,
        &similarity,
        &mut BezierAlgebraicCuspSemicircleSimilarityCache2::default(),
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
fn transform_retained_region_fragment(
    fragment: &BezierSplitFragment2,
    m00: &Real,
    m01: &Real,
    m10: &Real,
    m11: &Real,
    tx: &Real,
    ty: &Real,
    similarity: &std::cell::OnceCell<Option<crate::Similarity2>>,
    semicircle_similarity_cache: &mut BezierAlgebraicCuspSemicircleSimilarityCache2,
    policy: &CurveContext,
) -> ExactCurveResult<BezierSplitFragment2> {
    let similarity = similarity.get_or_init(|| {
        crate::Similarity2::try_from_real_affine(
            m00.clone(),
            m01.clone(),
            m10.clone(),
            m11.clone(),
            tx.clone(),
            ty.clone(),
        )
        .ok()
    });
    let retained_similarity = || {
        similarity.as_ref().ok_or_else(|| {
            ExactCurveError::blocked(
                CurveOperation2::Transformation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            )
        })
    };
    match fragment {
        BezierSplitFragment2::Materialized { start, end, curve } => {
            Ok(BezierSplitFragment2::Materialized {
                start: start.clone(),
                end: end.clone(),
                curve: transform_region_subcurve(
                    curve,
                    m00,
                    m01,
                    m10,
                    m11,
                    tx,
                    ty,
                    similarity.as_ref(),
                )?,
            })
        }
        BezierSplitFragment2::RetainedBezier {
            reversed,
            start,
            end,
            source_curve: source,
            ..
        } => {
            let source =
                transform_region_subcurve(source, m00, m01, m10, m11, tx, ty, similarity.as_ref())?;
            Ok(BezierSplitFragment2::RetainedBezier {
                reversed: *reversed,
                start: start.clone(),
                end: end.clone(),
                start_image: transform_region_endpoint_image(start, &source, policy),
                end_image: transform_region_endpoint_image(end, &source, policy),
                source_curve: source,
            })
        }
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            let parallel = fragment
                .parallel()
                .transform_similarity(retained_similarity()?)
                .map_err(affine_region_error)?;
            Ok(BezierSplitFragment2::AnalyticParallel(
                crate::BezierParallelFragment2::from_certified_range(
                    parallel,
                    fragment.range().clone(),
                    fragment.is_reversed(),
                ),
            ))
        }
        BezierSplitFragment2::AlgebraicChord(chord) => match if let Some(similarity) = similarity {
            semicircle_similarity_cache.chord(chord, similarity, policy)
        } else {
            chord.transform_affine(m00, m01, m10, m11, tx, ty, policy)
        }
        .map_err(affine_region_error)?
        {
            Classification::Decided(mut transformed) => {
                if let Some(similarity) = similarity.as_ref() {
                    let contacts = chord
                        .parallel_tangent_contacts()
                        .iter()
                        .map(|contact| contact.transform_similarity(similarity))
                        .collect::<CurveResult<Vec<_>>>()
                        .map_err(affine_region_error)?;
                    transformed = transformed.with_parallel_tangent_contacts(contacts);
                }
                Ok(BezierSplitFragment2::AlgebraicChord(transformed))
            }
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Transformation,
                CurveFamily2::RationalBezier,
                reason,
            )),
        },
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            Ok(BezierSplitFragment2::AlgebraicCuspSemicircle(
                fragment
                    .transform_similarity_cached(
                        retained_similarity()?,
                        semicircle_similarity_cache,
                    )
                    .map_err(affine_region_error)?,
            ))
        }
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let transform = retained_similarity()?;
            let source = match fragment.source() {
                BezierSelectedFiberSource2::Rational(curve) => {
                    BezierSelectedFiberSource2::Rational(curve.transform_similarity(transform))
                }
                BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
                    BezierSelectedFiberSource2::AnalyticParallel(
                        parallel
                            .transform_similarity(transform)
                            .map_err(affine_region_error)?,
                    )
                }
            };
            let (source_start, source_end) = if fragment.is_reversed() {
                (fragment.end_point(), fragment.start_point())
            } else {
                (fragment.start_point(), fragment.end_point())
            };
            let transformed_start = CurvePoint2::from(crate::BezierSimilarityPoint2::new(
                source_start.clone(),
                transform.clone(),
                policy,
            ));
            let transformed_end = CurvePoint2::from(crate::BezierSimilarityPoint2::new(
                source_end.clone(),
                transform.clone(),
                policy,
            ));
            let transformed = BezierSplitFragment2::SelectedFiber(
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    source,
                    fragment.range().clone(),
                    transformed_start,
                    transformed_end,
                ),
            );
            if fragment.is_reversed() {
                transformed.reversed().map_err(affine_region_error)
            } else {
                Ok(transformed)
            }
        }
    }
}

fn transform_region_endpoint_image(
    parameter: &BezierParameter2,
    source: &BezierSubcurve2,
    policy: &CurveContext,
) -> Option<BezierAlgebraicEndpointImage2> {
    match parameter {
        BezierParameter2::Exact(_) => None,
        BezierParameter2::Algebraic(parameter) => Some(
            BezierAlgebraicEndpointImage2::from_source_curve_first_order(source, parameter, policy),
        ),
    }
}

#[allow(clippy::too_many_arguments)]
fn transform_region_subcurve(
    curve: &BezierSubcurve2,
    m00: &Real,
    m01: &Real,
    m10: &Real,
    m11: &Real,
    tx: &Real,
    ty: &Real,
    similarity: Option<&crate::Similarity2>,
) -> ExactCurveResult<BezierSubcurve2> {
    let point = |point: &Point2| affine_region_point(point, m00, m01, m10, m11, tx, ty);
    match curve {
        BezierSubcurve2::Quadratic(curve) => {
            if let Some(similarity) = similarity {
                return curve
                    .transform_similarity_with_retained_provenance(similarity)
                    .map(BezierSubcurve2::Quadratic)
                    .map_err(affine_region_error);
            }
            let start = point(curve.start());
            let control = point(curve.control());
            let end = point(curve.end());
            let transformed = if curve.retained_exact_line_image().is_some() {
                QuadraticBezier2::with_retained_exact_line_image(start, control, end)
                    .map_err(affine_region_error)?
            } else {
                QuadraticBezier2::new(start, control, end)
            };
            Ok(BezierSubcurve2::Quadratic(transformed))
        }
        BezierSubcurve2::Cubic(curve) => Ok(BezierSubcurve2::Cubic(CubicBezier2::new(
            point(curve.start()),
            point(curve.control1()),
            point(curve.control2()),
            point(curve.end()),
        ))),
        BezierSubcurve2::RationalQuadratic(curve) => Ok(BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(
                point(curve.start()),
                point(curve.control()),
                point(curve.end()),
                curve.start_weight().clone(),
                curve.control_weight().clone(),
                curve.end_weight().clone(),
            )
            .map_err(affine_region_error)?,
        )),
        BezierSubcurve2::Rational(curve) => Ok(BezierSubcurve2::Rational(
            curve.transformed_affine([m00, m01, m10, m11, tx, ty]),
        )),
    }
}

#[allow(clippy::too_many_arguments)]
fn affine_region_point(
    point: &Point2,
    m00: &Real,
    m01: &Real,
    m10: &Real,
    m11: &Real,
    tx: &Real,
    ty: &Real,
) -> Point2 {
    Point2::new(
        m00 * point.x() + m01 * point.y() + tx,
        m10 * point.x() + m11 * point.y() + ty,
    )
}

fn filled_sides_from_roles_and_areas(
    roles: &[CurveRegionLoopRole],
    signed_areas: &[Real],
    policy: &CurveContext,
) -> CurveResult<Vec<bool>> {
    if roles.len() != signed_areas.len() {
        return Err(CurveError::Topology(
            "curved-region role and orientation evidence counts differ".into(),
        ));
    }
    roles
        .iter()
        .zip(signed_areas)
        .map(|(role, area)| match real_sign(area, policy) {
            Some(RealSign::Positive) => Ok(*role == CurveRegionLoopRole::Material),
            Some(RealSign::Negative) => Ok(*role == CurveRegionLoopRole::Hole),
            Some(RealSign::Zero) => Err(CurveError::Topology(
                "curved-region boundary loop has zero signed area".into(),
            )),
            None => Err(CurveError::Topology(
                "curved-region boundary orientation could not be certified".into(),
            )),
        })
        .collect()
}

fn validate_unique_arrangement_source_indices(
    mut indices: Vec<usize>,
    error: &str,
) -> CurveResult<()> {
    indices.sort_unstable();
    if indices.windows(2).any(|window| window[0] == window[1]) {
        return Err(CurveError::Topology(error.into()));
    }
    Ok(())
}

fn retained_line_loop_to_contour(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<Contour2>> {
    let mut segments = Vec::with_capacity(boundary_loop.fragments().len());
    let mut blocker = None;
    for fragment in boundary_loop.fragments() {
        let endpoints = match retained_line_fragment_endpoints(fragment, policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                blocker.get_or_insert(reason);
                continue;
            }
        };
        let (start, end) = endpoints;
        segments.push(Segment2::Line(LineSeg2::try_new(start, end)?));
    }
    if let Some(reason) = blocker {
        return Ok(Classification::Uncertain(reason));
    }
    Contour2::try_new(segments).map(Classification::Decided)
}

/// Returns exact line-segment endpoints for a retained line-image fragment.
///
/// Materialized fragments must carry a certified exact endpoint line-image
/// fit. Algebraic endpoint-image fragments are accepted only when the endpoint
/// point evidence has exact point witnesses, or when an exact boundary
/// parameter can be replayed against the retained source curve. This follows
/// the exactness model's retained-object discipline: algebraic endpoints become
/// line-contour topology only through exact construction evidence, not by
/// sampling isolating intervals. The native fit certificate proves every
/// control point lies on the endpoint segment, preserving the exact
/// object/predicate split while allowing non-affine parameterizations whose
/// image is still exactly one line segment.
fn retained_line_fragment_endpoints(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<(Point2, Point2)>> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            let fit = match subcurve_fit_exact_line_image(curve, policy)? {
                Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => fit,
                Classification::Decided(BezierLineImageFitRelation::NotLine) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(Classification::Decided((
                fit.line().start().clone(),
                fit.line().end().clone(),
            )))
        }
        BezierSplitFragment2::RetainedBezier {
            reversed,
            start,
            end,
            source_curve,
            start_image,
            end_image,
        } => {
            match subcurve_fit_exact_line_image(source_curve, policy)? {
                Classification::Decided(BezierLineImageFitRelation::Fit(_)) => {}
                Classification::Decided(BezierLineImageFitRelation::NotLine) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let start = match retained_line_endpoint_point(
                start,
                start_image.as_ref(),
                source_curve,
                policy,
            ) {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let end =
                match retained_line_endpoint_point(end, end_image.as_ref(), source_curve, policy) {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let points = if *reversed {
                (end, start)
            } else {
                (start, end)
            };
            Ok(Classification::Decided(points))
        }
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            let relation = match fragment.parallel().source() {
                crate::BezierParallelSource2::Quadratic(source) => {
                    source.fit_exact_line_image(policy)?
                }
                crate::BezierParallelSource2::Cubic(source) => {
                    source.fit_exact_line_image(policy)?
                }
                crate::BezierParallelSource2::Rational(source) => {
                    source.fit_exact_line_image(policy)?
                }
            };
            match relation {
                Classification::Decided(BezierLineImageFitRelation::Fit(_)) => {}
                Classification::Decided(BezierLineImageFitRelation::NotLine) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let Some((start_parameter, end_parameter)) = fragment.range().scalar_endpoints() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            };
            let start = match fragment.parallel().point_at(start_parameter, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let end = match fragment.parallel().point_at(end_parameter, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(Classification::Decided(if fragment.is_reversed() {
                (end, start)
            } else {
                (start, end)
            }))
        }
        BezierSplitFragment2::AlgebraicChord(chord) => {
            let line = if let Some(line) = chord.exact_line() {
                line
            } else {
                let start = match retained_native_line_point(chord.start(), policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let end = match retained_native_line_point(chord.end(), policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                LineSeg2::try_new(start, end)?
            };
            Ok(Classification::Decided((
                line.start().clone(),
                line.end().clone(),
            )))
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let Some(source) = fragment.rational_curve() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            // The source's line-image and monotonicity certificates cover its
            // unit chart. A restriction inside that chart has exactly the
            // segment between its endpoints; collinearity alone would miss
            // excursions beyond the endpoints of a nonmonotone restriction.
            for (endpoint, limit, outside) in [
                (
                    fragment.range().start(),
                    Real::zero(),
                    std::cmp::Ordering::Less,
                ),
                (
                    fragment.range().end(),
                    Real::one(),
                    std::cmp::Ordering::Greater,
                ),
            ] {
                match endpoint.cmp_by_refinement(&limit.into(), policy)? {
                    Classification::Decided(order) if order == outside => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    Classification::Decided(_) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let line = match source.fit_exact_line_image(policy)? {
                Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => fit,
                Classification::Decided(BezierLineImageFitRelation::NotLine) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let axis = match compare_reals(line.line().start().x(), line.line().end().x(), policy) {
                Some(std::cmp::Ordering::Equal) => crate::Axis2::Y,
                Some(_) => crate::Axis2::X,
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            match source.axis_monotonicity_classified(axis, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
            let start = match retained_native_line_point(fragment.start_point(), policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            Ok(retained_native_line_point(fragment.end_point(), policy)?.map(|end| (start, end)))
        }
    }
}

/// Reuses exact scalar witnesses while retaining selected endpoint authority.
fn retained_native_line_point(
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<Point2>> {
    let represented = match point {
        CurvePoint2(CurvePointData2::Exact(point)) => Classification::Decided(Some(point.clone())),
        CurvePoint2(CurvePointData2::Algebraic(point)) => {
            Classification::Decided(point.exact_point(policy))
        }
        CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => {
            point.exact_represented_point(policy)?
        }
        CurvePoint2(CurvePointData2::Endpoint(endpoint)) => match endpoint.resolve(policy)? {
            Classification::Decided(Some(point)) => {
                return retained_native_line_point(&point, policy);
            }
            Classification::Decided(None) => Classification::Decided(None),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        _ => Classification::Decided(None),
    };
    Ok(match represented {
        Classification::Decided(Some(point)) => Classification::Decided(point),
        Classification::Decided(None) => Classification::Uncertain(UncertaintyReason::Unsupported),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

pub(crate) fn retained_line_fragment_segment(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<LineSeg2>> {
    let endpoints = match retained_line_fragment_endpoints(fragment, policy)? {
        Classification::Decided(endpoints) => endpoints,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    LineSeg2::try_new(endpoints.0, endpoints.1).map(Classification::Decided)
}

fn subcurve_fit_exact_line_image(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierLineImageFitRelation>> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.fit_exact_line_image(policy),
        BezierSubcurve2::Cubic(curve) => curve.fit_exact_line_image(policy),
        BezierSubcurve2::RationalQuadratic(curve) => curve.fit_exact_line_image(policy),
        BezierSubcurve2::Rational(curve) => curve.fit_exact_line_image(policy),
    }
}

fn retained_line_endpoint_point(
    parameter: &BezierParameter2,
    image: Option<&crate::BezierAlgebraicEndpointImage2>,
    source_curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> Classification<Point2> {
    match parameter {
        BezierParameter2::Exact(value) => subcurve_point_at(source_curve, value.clone(), policy),
        BezierParameter2::Algebraic(_) => {
            let Some(image) = image else {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            };
            let point = match image.point() {
                Ok(Classification::Decided(point)) => point,
                Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
                Err(_) => return Classification::Uncertain(UncertaintyReason::Boundary),
            };
            match exact_point_from_image(point, Some(policy)) {
                Some(point) => Classification::Decided(point),
                None => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
    }
}

fn exact_point_from_image(
    point: &RationalBezierAlgebraicPointImage2,
    resolution_policy: Option<&CurveContext>,
) -> Option<Point2> {
    point.exact_point(resolution_policy.unwrap_or(&CurveContext::STRICT))
}

fn retained_loop_to_native(
    boundary_loop: &CurveRegionBoundaryLoop2,
) -> Option<BezierBoundaryLoop2> {
    let mut fragments = Vec::with_capacity(boundary_loop.fragments().len());
    for fragment in boundary_loop.fragments() {
        let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
            return None;
        };
        fragments.push(curve.clone());
    }
    Some(BezierBoundaryLoop2 { fragments })
}

fn retained_native_loop_to_contour(
    boundary_loop: &CurveRegionBoundaryLoop2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<Contour2>> {
    let mut segments = Vec::with_capacity(boundary_loop.len());
    for fragment in boundary_loop.fragments() {
        let segment = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                materialized_native_subcurve_segment(curve, policy)?
            }
            _ => retained_line_fragment_segment(fragment, policy)?.map(Segment2::Line),
        };
        match segment {
            Classification::Decided(segment) => segments.push(segment),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(
        Contour2::from_validated_closed_segments(segments, fill_rule),
    ))
}

pub(crate) fn materialized_native_subcurve_segment(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> CurveResult<Classification<Segment2>> {
    if let BezierSubcurve2::Quadratic(curve) = curve
        && let Some(line) = curve.retained_exact_line_image()
    {
        return Ok(Classification::Decided(Segment2::Line(line.clone())));
    }
    if let BezierSubcurve2::RationalQuadratic(curve) = curve
        && curve.retained_circular_conic().is_some()
    {
        return crate::arc_bezier::rational_quadratic_circular_arc(curve, policy).map(
            |arc| match arc {
                Classification::Decided(Some(arc)) => Classification::Decided(Segment2::Arc(arc)),
                Classification::Decided(None) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        );
    }
    match subcurve_fit_exact_line_image(curve, policy)? {
        Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => {
            return Ok(Classification::Decided(Segment2::Line(fit.line().clone())));
        }
        Classification::Decided(BezierLineImageFitRelation::NotLine) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let arc = match curve {
        BezierSubcurve2::RationalQuadratic(curve) => {
            crate::arc_bezier::rational_quadratic_circular_arc(curve, policy)
        }
        BezierSubcurve2::Rational(curve) => {
            crate::arc_bezier::rational_bezier_circular_arc(curve, policy)
        }
        BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
    }?;
    Ok(match arc {
        Classification::Decided(Some(arc)) => Classification::Decided(Segment2::Arc(arc)),
        Classification::Decided(None) => Classification::Uncertain(UncertaintyReason::Unsupported),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

fn native_loop_sample_point(
    boundary_loop: &BezierBoundaryLoop2,
    policy: &CurveContext,
) -> Classification<Point2> {
    let Some(fragment) = boundary_loop.fragments().first() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let half = match Real::one() / Real::from(2_i8) {
        Ok(half) => half,
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    subcurve_point_at(fragment, half, policy)
}

fn retained_loop_sample_point_evidence(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    if boundary_loop.fragments().is_empty() {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let half = (Real::one() / Real::from(2_i8))?;
    let mut last_reason = UncertaintyReason::Unsupported;
    for fragment in boundary_loop.fragments() {
        let candidate = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                subcurve_point_at(curve, half.clone(), policy).map(CurvePoint2::from)
            }
            BezierSplitFragment2::RetainedBezier {
                start,
                end,
                source_curve,
                ..
            } => match start.strict_scalar_between(end, policy)? {
                Classification::Decided(parameter) => {
                    subcurve_point_at(source_curve, parameter, policy).map(CurvePoint2::from)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
            BezierSplitFragment2::AnalyticParallel(fragment) => fragment
                .representative_point(policy)?
                .map(CurvePoint2::from),
            BezierSplitFragment2::SelectedFiber(fragment) => fragment
                .representative_point(policy)?
                .map(CurvePoint2::from),
            BezierSplitFragment2::AlgebraicChord(chord) => chord.representative_point(policy)?,
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                match fragment.representative_point()? {
                    Classification::Decided(point) => {
                        Classification::Decided(CurvePoint2::from(point))
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
        };
        match candidate {
            Classification::Decided(point) => return Ok(Classification::Decided(point)),
            Classification::Uncertain(reason) => last_reason = reason,
        }
    }
    Ok(Classification::Uncertain(last_reason))
}

pub(crate) fn classify_point_evidence_against_retained_loop(
    region: &CurveRegion2,
    loop_index: usize,
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    let boundary_loop = region.data.boundary_loops.get(loop_index).ok_or_else(|| {
        CurveError::Topology("retained loop classification index is out of bounds".into())
    })?;
    let direct = match point {
        CurvePoint2(CurvePointData2::Exact(point)) => Some(classify_point_against_retained_loop(
            boundary_loop,
            point,
            policy,
        )?),
        CurvePoint2(CurvePointData2::Algebraic(point)) => {
            Some(match point.predicate_evaluator(policy)? {
                Classification::Decided(predicate) => {
                    if let Classification::Decided(bounds) =
                        retained_loop_query_bounds(boundary_loop, policy)
                        && algebraic_point_is_decided_outside_bounds(&predicate, &bounds, policy)?
                    {
                        Classification::Decided(ContourPointLocation::Outside)
                    } else {
                        classify_algebraic_point_against_retained_loop(
                            boundary_loop,
                            &predicate,
                            FillRule::EvenOdd,
                            true,
                            policy,
                        )?
                    }
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            })
        }
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
        | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
        | CurvePoint2(CurvePointData2::AnalyticParallel(_))
        | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
    };
    match direct {
        Some(decided @ Classification::Decided(_)) => Ok(decided),
        Some(Classification::Uncertain(_)) | None => {
            crate::curve_region_boolean::classify_retained_point_evidence_against_loop_by_probe(
                region,
                loop_index,
                point.clone(),
                policy,
            )
        }
    }
}

fn subcurve_control_hull_contains_point(
    curve: &BezierSubcurve2,
    point: &Point2,
    policy: &CurveContext,
) -> Classification<bool> {
    let bounds = match curve {
        BezierSubcurve2::Quadratic(curve) => Aabb2::from_points(curve.control_points()),
        BezierSubcurve2::Cubic(curve) => Aabb2::from_points(curve.control_points()),
        BezierSubcurve2::RationalQuadratic(curve) => {
            if curve.common_nonzero_weight_sign(policy).is_none() {
                return Classification::Uncertain(UncertaintyReason::RealSign);
            }
            Aabb2::from_points(curve.control_points())
        }
        BezierSubcurve2::Rational(_) => {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
    };
    match bounds {
        Classification::Decided(bounds) => bounds.contains_point(point, policy),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn algebraic_point_is_decided_outside_bounds(
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    bounds: &Aabb2,
    policy: &CurveContext,
) -> CurveResult<bool> {
    for (use_x, minimum, maximum) in [
        (true, bounds.min_x(), bounds.max_x()),
        (false, bounds.min_y(), bounds.max_y()),
    ] {
        if matches!(
            point.coordinate_order_to_real(use_x, minimum, policy)?,
            Classification::Decided(std::cmp::Ordering::Less)
        ) || matches!(
            point.coordinate_order_to_real(use_x, maximum, policy)?,
            Classification::Decided(std::cmp::Ordering::Greater)
        ) {
            return Ok(true);
        }
    }
    Ok(false)
}

fn classify_algebraic_point_against_line_loop(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    let mut winding = 0_i32;
    for fragment in boundary_loop.fragments() {
        let line = match retained_line_fragment_segment(fragment, policy)? {
            Classification::Decided(line) => line,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let start_order = match point.coordinate_order_to_real(false, line.start().y(), policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_order = match point.coordinate_order_to_real(false, line.end().y(), policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if matches!(
            (start_order, end_order),
            (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
        ) {
            continue;
        }
        let side = match point.oriented_line_side(line.start(), line.end(), policy)? {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if side == LineSide::On {
            let start_x = match point.coordinate_order_to_real(true, line.start().x(), policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let end_x = match point.coordinate_order_to_real(true, line.end().x(), policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if !matches!(
                (start_x, end_x),
                (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                    | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
            ) {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            continue;
        }
        if start_order != std::cmp::Ordering::Less
            && end_order == std::cmp::Ordering::Less
            && side == LineSide::Left
        {
            winding += 1;
        } else if start_order == std::cmp::Ordering::Less
            && end_order != std::cmp::Ordering::Less
            && side == LineSide::Right
        {
            winding -= 1;
        }
    }
    Ok(Classification::Decided(winding_location(
        winding, fill_rule,
    )))
}

#[derive(Clone)]
struct AlgebraicRayHomogeneousControl2 {
    x: Real,
    y: Real,
    weight: Real,
}

struct AlgebraicRayRationalFragment2 {
    curve: RationalBezier2,
    retained_range: Option<CurveParameterRange2>,
    reversed: bool,
}

enum AlgebraicRayRetainedFragment2 {
    Rational(AlgebraicRayRationalFragment2),
    AnalyticParallel(crate::bezier_offset::BezierParallelAlgebraicRay2),
    AlgebraicChord(crate::BezierAlgebraicChord2),
    AlgebraicCusp(crate::bezier_offset::BezierAlgebraicCuspSemicircleAlgebraicRay2),
}

#[derive(Default)]
struct AlgebraicRaySignHull2 {
    negative: bool,
    zero: bool,
    positive: bool,
    first: Option<RealSign>,
    last: Option<RealSign>,
}

impl AlgebraicRaySignHull2 {
    fn include(&mut self, sign: RealSign) {
        match sign {
            RealSign::Negative => self.negative = true,
            RealSign::Zero => self.zero = true,
            RealSign::Positive => self.positive = true,
        }
        self.first.get_or_insert(sign);
        self.last = Some(sign);
    }
}

fn classify_algebraic_point_against_retained_loop(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    fill_rule: FillRule,
    certify_boundary: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    match classify_algebraic_point_against_line_loop(boundary_loop, point, fill_rule, policy)? {
        decided @ Classification::Decided(_) => return Ok(decided),
        Classification::Uncertain(UncertaintyReason::Unsupported) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    classify_algebraic_point_against_retained_loop_with_cusps(
        boundary_loop,
        point,
        fill_rule,
        certify_boundary,
        policy,
    )
}

fn classify_algebraic_point_against_retained_loop_with_cusps(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    fill_rule: FillRule,
    certify_boundary: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    let fragments = match prepare_algebraic_ray_retained_fragments(boundary_loop, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };

    if certify_boundary {
        for fragment in &fragments {
            let contains = match fragment {
                AlgebraicRayRetainedFragment2::Rational(fragment) => {
                    algebraic_point_on_rational_fragment(fragment, point, policy)?
                }
                AlgebraicRayRetainedFragment2::AnalyticParallel(fragment) => {
                    fragment.contains_point(point, None, policy)?
                }
                AlgebraicRayRetainedFragment2::AlgebraicChord(fragment) => {
                    fragment.contains_algebraic_point(point, policy)?
                }
                AlgebraicRayRetainedFragment2::AlgebraicCusp(fragment) => {
                    fragment.contains_point(point, policy)?
                }
            };
            match contains {
                Classification::Decided(true) => {
                    return Ok(Classification::Decided(ContourPointLocation::Boundary));
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }

    // There are at most two endpoint-collinear slopes per retained fragment.
    // Testing 2n+1 exact integer slopes therefore finds a nondegenerate ray
    // whenever the promised off-boundary query is distinct from every vertex.
    let candidate_count = fragments.len().saturating_mul(2).saturating_add(1);
    let mut last_reason = UncertaintyReason::Predicate;
    for slope in 0..candidate_count {
        let Ok(slope) = u64::try_from(slope) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let direction_x = Real::one();
        let direction_y = Real::from(slope);
        let side_x = -direction_y.clone();
        let side_y = direction_x.clone();
        let mut admissible = match algebraic_ray_retained_fragments_admit_direction(
            &fragments, point, &side_x, &side_y, policy,
        )? {
            Classification::Decided(admissible) => admissible,
            Classification::Uncertain(reason) => {
                last_reason = reason;
                false
            }
        };
        if !admissible {
            continue;
        }

        let winding = match algebraic_ray_retained_fragments_winding(
            &fragments,
            point,
            &direction_x,
            &direction_y,
            None,
            false,
            policy,
        )? {
            Classification::Decided(winding) => winding,
            Classification::Uncertain(reason) => {
                last_reason = reason;
                admissible = false;
                0
            }
        };
        if !admissible {
            continue;
        }
        return Ok(Classification::Decided(winding_location(
            winding, fill_rule,
        )));
    }
    Ok(Classification::Uncertain(last_reason))
}

fn prepare_algebraic_ray_retained_fragments(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<AlgebraicRayRetainedFragment2>>> {
    let mut fragments = Vec::with_capacity(boundary_loop.fragments().len());
    for fragment in boundary_loop.fragments() {
        match fragment {
            BezierSplitFragment2::AlgebraicChord(chord) => {
                if chord.exact_line().is_some() {
                    let fragment = match retained_fragment_algebraic_ray_curve(fragment, policy)? {
                        Classification::Decided(fragment) => fragment,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    fragments.push(AlgebraicRayRetainedFragment2::Rational(fragment));
                    continue;
                }
                chord.validate_policy(policy)?;
                fragments.push(AlgebraicRayRetainedFragment2::AlgebraicChord(chord.clone()));
            }
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                let evaluator = match fragment.algebraic_ray_evaluator(policy)? {
                    Classification::Decided(evaluator) => evaluator,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                fragments.push(AlgebraicRayRetainedFragment2::AlgebraicCusp(evaluator));
            }
            _ => match retained_fragment_algebraic_ray_curve(fragment, policy)? {
                Classification::Decided(fragment) => {
                    fragments.push(AlgebraicRayRetainedFragment2::Rational(fragment));
                }
                Classification::Uncertain(UncertaintyReason::Unsupported) => {
                    let fragment =
                        match retained_fragment_analytic_algebraic_ray_curve(fragment, policy)? {
                            Classification::Decided(fragment) => fragment,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    fragments.push(AlgebraicRayRetainedFragment2::AnalyticParallel(fragment));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        }
    }
    Ok(Classification::Decided(fragments))
}

fn algebraic_ray_retained_fragments_admit_direction(
    fragments: &[AlgebraicRayRetainedFragment2],
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    side_x: &Real,
    side_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    for fragment in fragments {
        match fragment {
            // Rational winding owns finite endpoints spatially and can use
            // a vertex ray without reconstructing its endpoint images.
            AlgebraicRayRetainedFragment2::Rational(_) => {}
            AlgebraicRayRetainedFragment2::AnalyticParallel(fragment) => {
                match fragment.endpoint_side_signs(point, side_x, side_y, policy)? {
                    Classification::Decided(signs)
                        if signs.into_iter().all(|sign| sign != RealSign::Zero) => {}
                    Classification::Decided(_) => return Ok(Classification::Decided(false)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            AlgebraicRayRetainedFragment2::AlgebraicChord(fragment) => {
                match fragment.algebraic_ray_endpoint_side_signs(point, side_x, side_y, policy)? {
                    Classification::Decided(signs)
                        if signs.into_iter().all(|sign| sign != RealSign::Zero) => {}
                    Classification::Decided(_) => return Ok(Classification::Decided(false)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            AlgebraicRayRetainedFragment2::AlgebraicCusp(fragment) => {
                match fragment.endpoint_side_signs(point, side_x, side_y, policy)? {
                    Classification::Decided(signs)
                        if signs.into_iter().all(|sign| sign != RealSign::Zero) => {}
                    Classification::Decided(_) => return Ok(Classification::Decided(false)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
    }
    Ok(Classification::Decided(true))
}

fn algebraic_ray_retained_fragments_winding(
    fragments: &[AlgebraicRayRetainedFragment2],
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    direction_x: &Real,
    direction_y: &Real,
    skipped_fragment: Option<usize>,
    skip_incident_origin_contacts: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<i32>> {
    if let Some(fragment_index) = skipped_fragment {
        match fragments.get(fragment_index) {
            Some(
                AlgebraicRayRetainedFragment2::AlgebraicCusp(_)
                | AlgebraicRayRetainedFragment2::AlgebraicChord(_),
            ) => {}
            Some(
                AlgebraicRayRetainedFragment2::Rational(_)
                | AlgebraicRayRetainedFragment2::AnalyticParallel(_),
            ) => {
                return Err(CurveError::Topology(
                    "an algebraic side ray can skip only a retained algebraic source".into(),
                ));
            }
            None => {
                return Err(CurveError::Topology(
                    "the algebraic side-ray source fragment is missing".into(),
                ));
            }
        }
    }
    let mut winding = 0_i32;
    for fragment in fragments {
        let delta = match fragment {
            AlgebraicRayRetainedFragment2::Rational(fragment) => {
                if skip_incident_origin_contacts {
                    match algebraic_point_rational_curve_ray_winding_skipping_incident_origin(
                        fragment,
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                        Classification::Decided(Some(delta)) => Classification::Decided(delta),
                        Classification::Decided(None) => {
                            algebraic_point_rational_curve_ray_winding(
                                fragment,
                                point,
                                direction_x,
                                direction_y,
                                policy,
                            )?
                        }
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                } else {
                    algebraic_point_rational_curve_ray_winding(
                        fragment,
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )?
                }
            }
            AlgebraicRayRetainedFragment2::AnalyticParallel(fragment) => {
                if skip_incident_origin_contacts {
                    match fragment.forward_ray_winding_delta_skipping_incident_origin(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                        Classification::Decided(Some(delta)) => Classification::Decided(delta),
                        Classification::Decided(None) => fragment.forward_ray_winding_delta(
                            point,
                            direction_x,
                            direction_y,
                            policy,
                        )?,
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                } else {
                    fragment.forward_ray_winding_delta(point, direction_x, direction_y, policy)?
                }
            }
            AlgebraicRayRetainedFragment2::AlgebraicChord(fragment) => {
                if skip_incident_origin_contacts {
                    match fragment.algebraic_forward_ray_winding_delta_skipping_incident_origin(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                        Classification::Decided(Some(delta)) => Classification::Decided(delta),
                        Classification::Decided(None) => fragment
                            .algebraic_forward_ray_winding_delta(
                                point,
                                direction_x,
                                direction_y,
                                policy,
                            )?,
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                } else {
                    fragment.algebraic_forward_ray_winding_delta(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )?
                }
            }
            AlgebraicRayRetainedFragment2::AlgebraicCusp(fragment) => {
                if skip_incident_origin_contacts {
                    match fragment.forward_ray_winding_delta_skipping_incident_origin(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                        Classification::Decided(Some(delta)) => Classification::Decided(delta),
                        Classification::Decided(None) => fragment.forward_ray_winding_delta(
                            point,
                            direction_x,
                            direction_y,
                            false,
                            policy,
                        )?,
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                } else {
                    fragment.forward_ray_winding_delta(
                        point,
                        direction_x,
                        direction_y,
                        false,
                        policy,
                    )?
                }
            }
        };
        let delta = match delta {
            Classification::Decided(delta) => delta,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        winding = winding.checked_add(delta).ok_or_else(|| {
            CurveError::Topology("algebraic ray winding exceeds the region counter".into())
        })?;
    }
    Ok(Classification::Decided(winding))
}

fn retained_fragment_algebraic_ray_endpoints(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<[CurvePoint2; 2]> {
    let endpoint = |start_endpoint| -> CurveResult<_> {
        match curve_fragment_endpoint_point(fragment, start_endpoint, policy)? {
            Classification::Decided(Some(point)) => Ok(point),
            Classification::Decided(None) | Classification::Uncertain(_) => {
                Err(CurveError::Topology(
                    "a retained algebraic-ray fragment lost exact endpoint evidence".into(),
                ))
            }
        }
    };
    Ok([endpoint(true)?, endpoint(false)?])
}

fn retained_fragment_analytic_algebraic_ray_curve(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<crate::bezier_offset::BezierParallelAlgebraicRay2>> {
    let (parallel, range, reversed) = match fragment {
        BezierSplitFragment2::AnalyticParallel(fragment) => (
            fragment.parallel().clone(),
            CurveParameterRange2::from_bezier_range(fragment.range().clone()),
            fragment.is_reversed(),
        ),
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let Some(parallel) = fragment.analytic_parallel() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            (
                parallel.clone(),
                fragment.range().clone(),
                fragment.is_reversed(),
            )
        }
        _ => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
    };
    crate::bezier_offset::BezierParallelAlgebraicRay2::try_new(
        parallel,
        range,
        reversed,
        retained_fragment_algebraic_ray_endpoints(fragment, policy)?,
        policy,
    )
}

fn retained_fragment_algebraic_ray_curve(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<AlgebraicRayRationalFragment2>> {
    match retained_line_fragment_segment(fragment, policy)? {
        Classification::Decided(line) => {
            return Ok(Classification::Decided(AlgebraicRayRationalFragment2 {
                curve: RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(line),
                ))?,
                retained_range: None,
                reversed: false,
            }));
        }
        Classification::Uncertain(UncertaintyReason::Unsupported) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }

    let (curve, retained_range, reversed) = match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            (RationalBezier2::try_from_subcurve(curve)?, None, false)
        }
        BezierSplitFragment2::RetainedBezier {
            start,
            end,
            reversed,
            source_curve: curve,
            ..
        } => (
            RationalBezier2::try_from_subcurve(curve)?,
            Some(CurveParameterRange2::new_validated(
                CurveParameter2::from(start.clone()),
                CurveParameter2::from(end.clone()),
            )),
            *reversed,
        ),
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            let curve = match fragment
                .parallel()
                .exact_rational_parallel_component(policy)?
            {
                Classification::Decided(Some(curve)) => curve,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            (
                curve,
                Some(CurveParameterRange2::from_bezier_range(
                    fragment.range().clone(),
                )),
                fragment.is_reversed(),
            )
        }
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let Some(curve) = fragment.rational_curve() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            (
                curve.clone(),
                Some(fragment.range().clone()),
                fragment.is_reversed(),
            )
        }
        BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
    };
    // Native subdivision is an accelerator only inside its authored unit
    // domain. Exterior intervals retain the source and their exact bounds.
    let (curve, retained_range) = if let Some(range) = retained_range {
        if let Some((start, end)) = range.scalar_endpoints()
            && policy.strict_predicate_pass(|| {
                crate::classify::in_closed_unit_interval(start, policy) == Some(true)
                    && crate::classify::in_closed_unit_interval(end, policy) == Some(true)
            })
        {
            let curve = match curve.subcurve_between_exact(start, end, policy)? {
                Classification::Decided(curve) => curve,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            (curve, None)
        } else {
            (curve, Some(range))
        }
    } else {
        (curve, None)
    };
    Ok(Classification::Decided(AlgebraicRayRationalFragment2 {
        curve,
        retained_range,
        reversed,
    }))
}

fn algebraic_point_rational_curve_linear_equation(
    curve: &RationalBezier2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    x_factor: &Real,
    y_factor: &Real,
) -> CurveResult<BivariatePolynomial> {
    let power = curve.homogeneous_power_basis()?;
    let (query_x, query_y, query_weight) = point.coordinate_polynomials();
    let first_count = query_x.len().max(query_y.len()).max(query_weight.len());
    let second_count = power
        .x_numerator
        .len()
        .max(power.y_numerator.len())
        .max(power.weight.len());
    let query_linear = (0..first_count)
        .map(|index| {
            x_factor * query_x.get(index).cloned().unwrap_or_else(Real::zero)
                + y_factor * query_y.get(index).cloned().unwrap_or_else(Real::zero)
        })
        .collect::<Vec<_>>();
    let curve_linear = (0..second_count)
        .map(|index| {
            x_factor
                * power
                    .x_numerator
                    .get(index)
                    .cloned()
                    .unwrap_or_else(Real::zero)
                + y_factor
                    * power
                        .y_numerator
                        .get(index)
                        .cloned()
                        .unwrap_or_else(Real::zero)
        })
        .collect::<Vec<_>>();
    Ok(BivariatePolynomial::new(
        (0..first_count)
            .map(|first_power| {
                (0..second_count)
                    .map(|second_power| {
                        query_weight
                            .get(first_power)
                            .cloned()
                            .unwrap_or_else(Real::zero)
                            * &curve_linear[second_power]
                            - &query_linear[first_power]
                                * power
                                    .weight
                                    .get(second_power)
                                    .cloned()
                                    .unwrap_or_else(Real::zero)
                    })
                    .collect()
            })
            .collect(),
    ))
}

fn algebraic_point_on_rational_curve(
    curve: &RationalBezier2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let weight_sign = match curve.denominator_sign(&CurveParameterRange2::unit()) {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(_) => RealSign::Zero,
    };
    if weight_sign != RealSign::Zero {
        let controls = curve
            .homogeneous_controls()
            .iter()
            .map(|control| AlgebraicRayHomogeneousControl2 {
                x: control.x().clone(),
                y: control.y().clone(),
                weight: control.weight().clone(),
            })
            .collect::<Vec<_>>();
        for (factor_x, factor_y) in [(Real::one(), Real::zero()), (Real::zero(), Real::one())] {
            if let Classification::Decided(hull) = algebraic_ray_control_sign_hull(
                &controls,
                point,
                &factor_x,
                &factor_y,
                weight_sign,
                policy,
            )? && !hull.zero
                && (hull.negative ^ hull.positive)
            {
                return Ok(Classification::Decided(false));
            }
        }
    }
    let x =
        algebraic_point_rational_curve_linear_equation(curve, point, &Real::one(), &Real::zero())?;
    let y =
        algebraic_point_rational_curve_linear_equation(curve, point, &Real::zero(), &Real::one())?;
    let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
        &x,
        &y,
        CurveResultantParameter::First,
        point.retained_root(),
        &Real::zero(),
        &Real::one(),
        policy.predicate_policy(),
    );
    Ok(match report.status {
        AlgebraicFiberRootCountStatus::Counted => {
            Classification::Decided(report.distinct_root_count.unwrap_or(0) != 0)
        }
        AlgebraicFiberRootCountStatus::IdenticallyZeroFiber
        | AlgebraicFiberRootCountStatus::EndpointRoot => Classification::Decided(true),
        AlgebraicFiberRootCountStatus::UnsupportedCoefficient => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        AlgebraicFiberRootCountStatus::Undecided => {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
        AlgebraicFiberRootCountStatus::InvalidEvidence => {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }
        AlgebraicFiberRootCountStatus::InvalidInterval => {
            return Err(CurveError::Topology(
                "algebraic boundary incidence received an invalid unit interval".into(),
            ));
        }
    })
}

fn algebraic_point_on_rational_fragment(
    fragment: &AlgebraicRayRationalFragment2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let Some(range) = fragment.retained_range.as_ref() else {
        return algebraic_point_on_rational_curve(&fragment.curve, point, policy);
    };
    let x = algebraic_point_rational_curve_linear_equation(
        &fragment.curve,
        point,
        &Real::one(),
        &Real::zero(),
    )?;
    let y = algebraic_point_rational_curve_linear_equation(
        &fragment.curve,
        point,
        &Real::zero(),
        &Real::one(),
    )?;
    let mut identically_zero_count = 0_usize;
    let mut last_reason = UncertaintyReason::Predicate;
    for (incidence, predicate) in [(&x, &y), (&y, &x)] {
        let parameters = match selected_fiber_parameters(
            incidence,
            point.retained_parameter(),
            range,
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
                parameters
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                identically_zero_count += 1;
                continue;
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => continue,
            Classification::Uncertain(reason) => {
                last_reason = reason;
                continue;
            }
        };
        for parameter in parameters {
            match retained_curve_region_parameter_contains(&parameter, range, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            match algebraic_selected_correlated_predicate_sign(
                incidence,
                predicate,
                point.retained_parameter(),
                &parameter,
                policy,
            )? {
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Decided(true));
                }
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        return Ok(Classification::Decided(false));
    }
    if identically_zero_count == 2 {
        Ok(Classification::Decided(true))
    } else {
        Ok(Classification::Uncertain(last_reason))
    }
}

fn algebraic_point_rational_curve_ray_winding(
    fragment: &AlgebraicRayRationalFragment2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<i32>> {
    if fragment.retained_range.is_some() {
        return algebraic_point_retained_rational_curve_ray_winding(
            fragment,
            point,
            direction_x,
            direction_y,
            false,
            policy,
        );
    }
    let weight_sign = match fragment
        .curve
        .denominator_sign(&CurveParameterRange2::unit())
    {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let controls = fragment
        .curve
        .homogeneous_controls()
        .iter()
        .map(|control| AlgebraicRayHomogeneousControl2 {
            x: control.x().clone(),
            y: control.y().clone(),
            weight: control.weight().clone(),
        })
        .collect::<Vec<_>>();
    let side_x = -direction_y.clone();
    let side_y = direction_x.clone();
    let half = (Real::one() / Real::from(2_i8))?;
    let mut stack = vec![controls];
    let mut winding = 0_i32;
    while let Some(controls) = stack.pop() {
        let side = algebraic_ray_control_sign_hull(
            &controls,
            point,
            &side_x,
            &side_y,
            weight_sign,
            policy,
        )?;
        let side = match side {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if !side.zero && (!side.negative || !side.positive) {
            continue;
        }
        let ahead = match algebraic_ray_control_sign_hull(
            &controls,
            point,
            direction_x,
            direction_y,
            weight_sign,
            policy,
        )? {
            Classification::Decided(ahead) => ahead,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if (side.first == Some(RealSign::Zero) && ahead.first == Some(RealSign::Zero))
            || (side.last == Some(RealSign::Zero) && ahead.last == Some(RealSign::Zero))
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        if !ahead.positive {
            continue;
        }
        if !side.negative && !side.positive {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        if !ahead.negative {
            // Every interior point lies strictly ahead. Spatial ownership
            // makes the sum of all contacts telescope to the endpoint signs,
            // including zero endpoints. The same rule cancels artificial
            // subdivision endpoints without a separate midpoint contact.
            let delta = spatial_ray_winding_delta(
                side.first == Some(RealSign::Positive),
                side.last == Some(RealSign::Positive),
                false,
                false,
            );
            winding = winding.checked_add(delta).ok_or_else(|| {
                CurveError::Topology("algebraic ray winding exceeds the curve counter".into())
            })?;
            continue;
        }
        let (left, right) = split_algebraic_ray_controls_at_half(&controls, &half);
        stack.push(right);
        stack.push(left);
    }
    Ok(Classification::Decided(if fragment.reversed {
        winding.checked_neg().ok_or_else(|| {
            CurveError::Topology("algebraic ray winding reversal overflowed".into())
        })?
    } else {
        winding
    }))
}

fn algebraic_point_rational_curve_ray_winding_skipping_incident_origin(
    fragment: &AlgebraicRayRationalFragment2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<i32>>> {
    match algebraic_point_on_rational_fragment(fragment, point, policy)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    if fragment.retained_range.is_some() {
        return algebraic_point_retained_rational_curve_ray_winding(
            fragment,
            point,
            direction_x,
            direction_y,
            true,
            policy,
        )
        .map(|delta| delta.map(Some));
    }
    let retained = AlgebraicRayRationalFragment2 {
        curve: fragment.curve.clone(),
        retained_range: Some(CurveParameterRange2::from_bezier_range(
            BezierParameterRange2::from_exact(Real::zero(), Real::one()),
        )),
        reversed: fragment.reversed,
    };
    algebraic_point_retained_rational_curve_ray_winding(
        &retained,
        point,
        direction_x,
        direction_y,
        true,
        policy,
    )
    .map(|delta| delta.map(Some))
}

fn algebraic_point_retained_rational_curve_ray_winding(
    fragment: &AlgebraicRayRationalFragment2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    direction_x: &Real,
    direction_y: &Real,
    skip_incident_origin: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<i32>> {
    let range = fragment
        .retained_range
        .as_ref()
        .expect("retained algebraic ray winding requires a retained range");
    let weight_sign = match fragment.curve.denominator_sign(range) {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let side_x = -direction_y.clone();
    let side_y = direction_x.clone();
    let incidence =
        algebraic_point_rational_curve_linear_equation(&fragment.curve, point, &side_x, &side_y)?;

    if let BezierParameter2::Algebraic(retained) = point.retained_parameter()
        && bivariate_fiber_strict_sign_on_parameter_range(&incidence, retained, range, policy)?
            .is_some()
    {
        return Ok(Classification::Decided(0));
    }

    let parameters =
        match selected_fiber_parameters(&incidence, point.retained_parameter(), range, policy)? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
                parameters
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    let ahead = algebraic_point_rational_curve_linear_equation(
        &fragment.curve,
        point,
        direction_x,
        direction_y,
    )?;
    let denominator_sign = product_sign(point.denominator_sign(), weight_sign);
    let mut winding = 0_i32;
    for parameter in parameters {
        let [start, end] = match retained_curve_region_parameter_orders(&parameter, range, policy)?
        {
            Classification::Decided(orders) => orders,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        use std::cmp::Ordering::{Equal, Greater, Less};
        if start == Less || end == Greater || (start == Equal && end == Equal) {
            continue;
        }

        let ahead_sign = match algebraic_selected_correlated_predicate_sign(
            &incidence,
            &ahead,
            point.retained_parameter(),
            &parameter,
            policy,
        )? {
            Classification::Decided(sign) => product_sign(sign, denominator_sign),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let at_origin = match ahead_sign {
            RealSign::Negative => continue,
            RealSign::Zero => {
                if !skip_incident_origin {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                true
            }
            RealSign::Positive => false,
        };

        let mut derivative = incidence.clone();
        let mut derivative_order = 0_usize;
        let (before_positive, after_positive) = loop {
            derivative_order += 1;
            derivative = algebraic_ray_bivariate_second_derivative(&derivative);
            let derivative_sign = match algebraic_selected_correlated_predicate_sign(
                &incidence,
                &derivative,
                point.retained_parameter(),
                &parameter,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if derivative_sign != RealSign::Zero {
                let after_positive =
                    product_sign(derivative_sign, denominator_sign) == RealSign::Positive;
                let before_positive = if derivative_order.is_multiple_of(2) {
                    after_positive
                } else {
                    !after_positive
                };
                break (before_positive, after_positive);
            }
            if derivative.coefficients.iter().all(|row| row.len() <= 1) {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
        };
        if at_origin {
            if before_positive == after_positive {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            continue;
        }
        let delta = spatial_ray_winding_delta(
            before_positive,
            after_positive,
            start == Equal,
            end == Equal,
        );
        winding = winding.checked_add(delta).ok_or_else(|| {
            CurveError::Topology("algebraic ray winding exceeds the curve counter".into())
        })?;
    }
    Ok(Classification::Decided(if fragment.reversed {
        winding.checked_neg().ok_or_else(|| {
            CurveError::Topology("algebraic ray winding reversal overflowed".into())
        })?
    } else {
        winding
    }))
}

fn algebraic_ray_bivariate_second_derivative(
    polynomial: &BivariatePolynomial,
) -> BivariatePolynomial {
    BivariatePolynomial::new(
        polynomial
            .coefficients
            .iter()
            .map(|row| {
                if row.len() <= 1 {
                    return vec![Real::zero()];
                }
                row.iter()
                    .enumerate()
                    .skip(1)
                    .map(|(power, coefficient)| coefficient * Real::from(power as u64))
                    .collect()
            })
            .collect(),
    )
}

fn algebraic_ray_control_sign_hull(
    controls: &[AlgebraicRayHomogeneousControl2],
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    x_factor: &Real,
    y_factor: &Real,
    weight_sign: RealSign,
    policy: &CurveContext,
) -> CurveResult<Classification<AlgebraicRaySignHull2>> {
    let mut hull = AlgebraicRaySignHull2::default();
    for control in controls {
        match point.homogeneous_linear_difference_sign(
            &control.x,
            &control.y,
            &control.weight,
            x_factor,
            y_factor,
            weight_sign,
            policy,
        )? {
            Classification::Decided(sign) => hull.include(sign),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(hull))
}

fn split_algebraic_ray_controls_at_half(
    controls: &[AlgebraicRayHomogeneousControl2],
    half: &Real,
) -> (
    Vec<AlgebraicRayHomogeneousControl2>,
    Vec<AlgebraicRayHomogeneousControl2>,
) {
    let mut level = controls.to_vec();
    let mut left = Vec::with_capacity(level.len());
    let mut right = Vec::with_capacity(level.len());
    left.push(level[0].clone());
    right.push(level[level.len() - 1].clone());
    for next_len in (1..level.len()).rev() {
        for index in 0..next_len {
            level[index] = AlgebraicRayHomogeneousControl2 {
                x: Real::dot2_refs([&level[index].x, &level[index + 1].x], [half, half]),
                y: Real::dot2_refs([&level[index].y, &level[index + 1].y], [half, half]),
                weight: Real::dot2_refs(
                    [&level[index].weight, &level[index + 1].weight],
                    [half, half],
                ),
            };
        }
        left.push(level[0].clone());
        right.push(level[next_len - 1].clone());
    }
    right.reverse();
    (left, right)
}

fn classify_point_against_native_loop_after_bounds(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    classify_point_against_native_loop_after_bounds_with_fill_rule(
        boundary_loop,
        point,
        FillRule::EvenOdd,
        policy,
    )
}

fn classify_point_against_native_loop_after_bounds_with_fill_rule(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    for fragment in boundary_loop.fragments() {
        if matches!(
            subcurve_control_hull_contains_point(fragment, point, policy),
            Classification::Decided(false)
        ) {
            continue;
        }
        match subcurve_contains_point(fragment, point, policy) {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            Classification::Decided(false) | Classification::Uncertain(_) => {}
        }
    }
    let rays = ray_candidates(point);
    let mut last_reason = UncertaintyReason::Boundary;
    for ray in rays {
        match classify_point_with_ray(boundary_loop, point, &ray, fill_rule, policy)? {
            Classification::Decided(location) => {
                return Ok(Classification::Decided(location));
            }
            Classification::Uncertain(reason) => last_reason = reason,
        }
    }
    Ok(Classification::Uncertain(last_reason))
}

fn classify_point_against_retained_loops(
    boundary_loops: &[CurveRegionBoundaryLoop2],
    point: &Point2,
    policy: &CurveContext,
    roles: Option<&[CurveRegionLoopRole]>,
    fill_rules: Option<&[FillRule]>,
) -> CurveResult<Classification<RegionPointLocation>> {
    if roles.is_some_and(|roles| roles.len() != boundary_loops.len())
        || fill_rules.is_some_and(|rules| rules.len() != boundary_loops.len())
    {
        return Err(CurveError::Topology(
            "retained region loop semantics are inconsistent with boundary loops".into(),
        ));
    }
    let mut inside = false;
    let mut signed_depth = 0_i32;
    for (index, boundary_loop) in boundary_loops.iter().enumerate() {
        let fill_rule = fill_rules.map_or(FillRule::EvenOdd, |rules| rules[index]);
        match classify_point_against_retained_loop_with_fill_rule(
            boundary_loop,
            point,
            fill_rule,
            policy,
        )? {
            Classification::Decided(ContourPointLocation::Inside) => {
                if let Some(roles) = roles {
                    signed_depth += match roles[index] {
                        CurveRegionLoopRole::Material => 1,
                        CurveRegionLoopRole::Hole => -1,
                    };
                } else {
                    inside = !inside;
                }
            }
            Classification::Decided(ContourPointLocation::Outside) => {}
            Classification::Decided(ContourPointLocation::Boundary) => {
                return Ok(Classification::Decided(RegionPointLocation::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let inside = roles.map_or(inside, |_| signed_depth > 0);
    Ok(Classification::Decided(if inside {
        RegionPointLocation::Inside
    } else {
        RegionPointLocation::Outside
    }))
}

fn classify_point_against_retained_loop(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    classify_point_against_retained_loop_with_fill_rule(
        boundary_loop,
        point,
        FillRule::EvenOdd,
        policy,
    )
}

fn classify_point_against_retained_loop_with_fill_rule(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &Point2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    if policy.strict_predicate_pass(|| {
        matches!(
            retained_loop_query_bounds(boundary_loop, policy),
            Classification::Decided(bounds)
                if bounds.contains_point(point, &CurveContext::STRICT)
                    == Classification::Decided(false)
        )
    }) {
        return Ok(Classification::Decided(ContourPointLocation::Outside));
    }
    for (fragment, evaluator) in boundary_loop
        .fragments()
        .iter()
        .zip(boundary_loop.rational_evaluators()?)
    {
        if let BezierSplitFragment2::Materialized { curve, .. } = fragment
            && matches!(
                subcurve_control_hull_contains_point(curve, point, policy),
                Classification::Decided(false)
            )
        {
            continue;
        }
        match policy.strict_predicate_pass(|| {
            retained_fragment_contains_point(fragment, evaluator.as_ref(), point, policy)
        })? {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            Classification::Decided(false) | Classification::Uncertain(_) => {}
        }
    }
    let mut last_reason = UncertaintyReason::Boundary;
    let rays = ray_candidates(point);
    // A ray that meets an unresolved endpoint is only one possible witness.
    // Exhaust every exact ray before permitting any terminal interpretation.
    for approximate in [false, true] {
        if approximate && !policy.permits_approximate_512() {
            break;
        }
        for ray in &rays {
            let classify =
                || classify_point_with_retained_ray(boundary_loop, point, ray, fill_rule, policy);
            let result = if approximate {
                classify()
            } else {
                policy.strict_predicate_pass(classify)
            }?;
            match result {
                Classification::Decided(location) => {
                    return Ok(Classification::Decided(location));
                }
                Classification::Uncertain(reason) => last_reason = reason,
            }
        }
    }
    Ok(Classification::Uncertain(last_reason))
}

fn retained_fragment_contains_point(
    fragment: &BezierSplitFragment2,
    evaluator: Option<&RationalBezier2>,
    point: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            Ok(subcurve_contains_point(curve, point, policy))
        }
        BezierSplitFragment2::RetainedBezier { start, end, .. } => {
            let Some(evaluator) = evaluator else {
                return Err(CurveError::Topology(
                    "retained algebraic source evaluator cache is incomplete".into(),
                ));
            };
            let range =
                CurveParameterRange2::new_validated(start.clone().into(), end.clone().into());
            Ok(evaluator
                .point_incidence_on_range(point, &range, policy)?
                .map(|incidence| match incidence {
                    RationalBezierPointIncidence2::EntireCurve => true,
                    RationalBezierPointIncidence2::Parameters(parameters) => !parameters.is_empty(),
                }))
        }
        BezierSplitFragment2::AnalyticParallel(fragment) => fragment.parallel().contains_point(
            point,
            &CurveParameterRange2::from_bezier_range(fragment.range().clone()),
            policy,
        ),
        BezierSplitFragment2::SelectedFiber(fragment) => {
            if let Some(curve) = fragment.rational_curve() {
                Ok(curve
                    .point_incidence_on_range(point, fragment.range(), policy)?
                    .map(|incidence| match incidence {
                        RationalBezierPointIncidence2::EntireCurve => true,
                        RationalBezierPointIncidence2::Parameters(parameters) => {
                            !parameters.is_empty()
                        }
                    }))
            } else {
                fragment
                    .analytic_parallel()
                    .expect("a selected-fiber source is rational or analytic")
                    .contains_point(point, fragment.range(), policy)
            }
        }
        BezierSplitFragment2::AlgebraicChord(chord) => {
            chord.contains_point(&CurvePoint2::from(point.clone()), policy)
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            fragment.contains_point(&CurvePoint2::from(point.clone()), policy)
        }
    }
}

#[derive(Clone, Copy)]
struct RetainedRayOriginContact<'a> {
    fragment_index: Option<usize>,
    parameter: Option<&'a CurveParameter2>,
    crossing_direction: BezierLineCrossingDirection,
    tangent_contacts: Option<&'a [crate::rational_bezier::RationalQuadraticCircleTangentContact2]>,
}

impl RetainedRayOriginContact<'_> {
    // The caller certifies the crossing in boundary traversal order. The
    // supporting-line solvers consume increasing source-parameter order.
    fn parameter_crossing_direction(&self, reversed: bool) -> BezierLineCrossingDirection {
        match (self.crossing_direction, reversed) {
            (BezierLineCrossingDirection::NegativeToPositive, true) => {
                BezierLineCrossingDirection::PositiveToNegative
            }
            (BezierLineCrossingDirection::PositiveToNegative, true) => {
                BezierLineCrossingDirection::NegativeToPositive
            }
            (direction, false) => direction,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RetainedRayWinding {
    Winding(i32),
    Boundary,
}

fn retained_circle_tangent_contacts(
    fragment: &BezierSplitFragment2,
) -> Option<&[crate::rational_bezier::RationalQuadraticCircleTangentContact2]> {
    let circle = match fragment {
        BezierSplitFragment2::Materialized {
            curve: BezierSubcurve2::RationalQuadratic(curve),
            ..
        } => curve.retained_circular_conic(),
        BezierSplitFragment2::Materialized {
            curve: BezierSubcurve2::Rational(curve),
            ..
        }
        | BezierSplitFragment2::RetainedBezier {
            source_curve: BezierSubcurve2::Rational(curve),
            ..
        } => curve.retained_circular_conic(),
        BezierSplitFragment2::RetainedBezier {
            source_curve: BezierSubcurve2::RationalQuadratic(curve),
            ..
        } => curve.retained_circular_conic(),
        BezierSplitFragment2::SelectedFiber(fragment) => fragment
            .rational_curve()
            .and_then(RationalBezier2::retained_circular_conic),
        BezierSplitFragment2::Materialized { .. }
        | BezierSplitFragment2::RetainedBezier { .. }
        | BezierSplitFragment2::AnalyticParallel(_)
        | BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
    }?;
    circle.tangent_contacts.as_deref()
}

fn classify_point_with_retained_ray(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &Point2,
    ray: &BezierRay2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    Ok(
        match classify_point_with_retained_ray_skipping_origin(
            boundary_loop,
            point,
            ray,
            None,
            policy,
        )? {
            Classification::Decided(RetainedRayWinding::Winding(winding)) => {
                Classification::Decided(winding_location(winding, fill_rule))
            }
            Classification::Decided(RetainedRayWinding::Boundary) => {
                Classification::Decided(ContourPointLocation::Boundary)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

fn classify_point_with_retained_ray_skipping_origin(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &Point2,
    ray: &BezierRay2,
    skipped_origin: Option<RetainedRayOriginContact<'_>>,
    policy: &CurveContext,
) -> CurveResult<Classification<RetainedRayWinding>> {
    let direction_x = &ray.direction_x;
    let direction_y = &ray.direction_y;
    let mut winding = 0_i32;
    let mut source_origin_contact_was_skipped = false;
    let cached = boundary_loop.fragment_query_bounds(policy);
    let cached_bounds = cached.map(|(bounds, _)| bounds);
    // A closed loop has zero winding at any point strictly outside its
    // certified bounding box, so its fragments need no ray query. The source
    // loop is excluded because the origin lies on it.
    if skipped_origin.is_none_or(|origin| origin.fragment_index.is_none())
        && let Some((_, Some(loop_box))) = cached
        && let (Some([x_low, x_high]), Some([y_low, y_high])) = (
            certified_f64_enclosure(point.x()),
            certified_f64_enclosure(point.y()),
        )
        && (x_high < loop_box[0]
            || x_low > loop_box[1]
            || y_high < loop_box[2]
            || y_low > loop_box[3])
    {
        return Ok(Classification::Decided(RetainedRayWinding::Winding(0)));
    }
    let approximate_ray = match (
        certified_f64_enclosure(point.x()),
        certified_f64_enclosure(point.y()),
        certified_f64_enclosure(direction_x),
        certified_f64_enclosure(direction_y),
    ) {
        (Some(x), Some(y), Some(dx), Some(dy)) => Some(([x, y], [dx, dy])),
        _ => None,
    };
    for (fragment_index, fragment) in boundary_loop.fragments().iter().enumerate() {
        let is_source_fragment =
            skipped_origin.is_some_and(|origin| origin.fragment_index == Some(fragment_index));
        if !is_source_fragment
            && let (Some(bounds), Some((origin, direction))) = (cached_bounds, approximate_ray)
            && let Some(approximate) = bounds[fragment_index].approximate
            && f64_box_certainly_misses_forward_ray(approximate, origin, direction)
        {
            continue;
        }
        let computed;
        let bounds = match cached_bounds {
            Some(bounds) => &bounds[fragment_index].exact,
            None => {
                computed = retained_fragment_query_bounds(fragment, policy);
                &computed
            }
        };
        if let Classification::Decided(bounds) = bounds
            && !retained_bounds_may_intersect_forward_ray(bounds, point, direction_x, direction_y)
            && !is_source_fragment
        {
            continue;
        }
        let exact_parallel_curve = match fragment {
            BezierSplitFragment2::AnalyticParallel(fragment) => match fragment
                .parallel()
                .exact_rational_parallel_component(policy)
            {
                Ok(Classification::Decided(Some(curve))) => Some(BezierSubcurve2::Rational(curve)),
                Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => None,
            },
            BezierSplitFragment2::Materialized { .. }
            | BezierSplitFragment2::RetainedBezier { .. }
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
            BezierSplitFragment2::SelectedFiber(_) => None,
        };
        if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
            let source_contact = skipped_origin.and_then(|origin| {
                (origin.fragment_index == Some(fragment_index))
                    .then_some(origin)
                    .and_then(|origin| {
                        Some((
                            origin.parameter?.as_algebraic_chord()?,
                            origin.crossing_direction,
                        ))
                    })
            });
            let result = match source_contact {
                Some((parameter, crossing_direction)) => {
                    let result = chord.forward_ray_winding_delta_skipping_origin(
                        point,
                        direction_x,
                        direction_y,
                        parameter,
                        crossing_direction,
                        policy,
                    )?;
                    if matches!(result, Classification::Decided(_)) {
                        source_origin_contact_was_skipped = true;
                    }
                    result
                }
                None if skipped_origin.is_some() => {
                    match chord.forward_ray_winding_delta_skipping_incident_origin(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                        Classification::Decided(Some(delta)) => {
                            if skipped_origin
                                .is_some_and(|origin| origin.fragment_index == Some(fragment_index))
                            {
                                source_origin_contact_was_skipped = true;
                            }
                            Classification::Decided(delta)
                        }
                        Classification::Decided(None) => chord.forward_ray_winding_delta(
                            point,
                            direction_x,
                            direction_y,
                            policy,
                        )?,
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                }
                None => chord.forward_ray_winding_delta(point, direction_x, direction_y, policy)?,
            };
            match result {
                Classification::Decided(delta) => winding += delta,
                Classification::Uncertain(UncertaintyReason::Boundary) => {
                    return Ok(Classification::Decided(RetainedRayWinding::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            continue;
        }
        if let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment {
            let source_contact = skipped_origin.and_then(|origin| {
                (origin.fragment_index == Some(fragment_index))
                    .then_some(origin)
                    .and_then(|origin| {
                        Some((
                            origin.parameter?.as_algebraic_cusp()?,
                            origin.crossing_direction,
                        ))
                    })
            });
            let result = match source_contact {
                Some((parameter, crossing_direction)) => {
                    let result = fragment.forward_ray_winding_delta_skipping_origin(
                        point,
                        direction_x,
                        direction_y,
                        parameter,
                        crossing_direction,
                        policy,
                    )?;
                    if matches!(result, Classification::Decided(_)) {
                        source_origin_contact_was_skipped = true;
                    }
                    result
                }
                None if skipped_origin.is_some() => match fragment
                    .forward_ray_winding_delta_skipping_incident_origin(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                    Classification::Decided(Some(delta)) => {
                        if skipped_origin
                            .is_some_and(|origin| origin.fragment_index == Some(fragment_index))
                        {
                            source_origin_contact_was_skipped = true;
                        }
                        Classification::Decided(delta)
                    }
                    Classification::Decided(None) => fragment.forward_ray_winding_delta(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )?,
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
                None => {
                    fragment.forward_ray_winding_delta(point, direction_x, direction_y, policy)?
                }
            };
            match result {
                Classification::Decided(delta) => winding += delta,
                Classification::Uncertain(UncertaintyReason::Boundary) => {
                    return Ok(Classification::Decided(RetainedRayWinding::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            continue;
        }
        let procedural_parallel = match fragment {
            BezierSplitFragment2::AnalyticParallel(fragment) if exact_parallel_curve.is_none() => {
                Some((
                    fragment.parallel(),
                    fragment.is_reversed(),
                    Some(fragment.range()),
                    None,
                ))
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                fragment.analytic_parallel().map(|parallel| {
                    (
                        parallel,
                        fragment.is_reversed(),
                        None,
                        Some(fragment.range()),
                    )
                })
            }
            BezierSplitFragment2::Materialized { .. }
            | BezierSplitFragment2::RetainedBezier { .. }
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
        };
        if let Some((parallel, reversed, ordinary_range, selected_range)) = procedural_parallel {
            let regular_range = selected_range.cloned().unwrap_or_else(|| {
                CurveParameterRange2::from_bezier_range(
                    ordinary_range
                        .expect("an ordinary analytic fragment retains its range")
                        .clone(),
                )
            });
            let certified_origin_parameter =
                skipped_origin.and_then(|origin| {
                    if origin.fragment_index == Some(fragment_index) {
                        return origin
                            .parameter
                            .and_then(CurveParameter2::as_bezier_parameter)
                            .and_then(BezierParameter2::scalar);
                    }
                    let source_fragment_index = origin.fragment_index?;
                    let fragment_count = boundary_loop.fragments().len();
                    let adjacent = fragment_count > 1
                        && ((source_fragment_index + 1) % fragment_count == fragment_index
                            || (fragment_index + 1) % fragment_count == source_fragment_index);
                    adjacent.then_some(())?;
                    origin.tangent_contacts?.iter().find_map(|contact| {
                        match contact {
                    crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(
                        contact,
                    ) if contact.parallel == *parallel && contact.point == *point => {
                        Some(&contact.parameter)
                    }
                    crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(_)
                    | crate::rational_bezier::RationalQuadraticCircleTangentContact2::Line {
                        ..
                    } => None,
                }
                    })
                });
            let certified_crossing = skipped_origin.and_then(|origin| {
                certified_origin_parameter
                    .map(|parameter| (parameter, origin.parameter_crossing_direction(reversed)))
            });
            let relation = match parallel.relation_to_supporting_line_on_regular_range(
                &ray.line,
                &regular_range,
                certified_crossing,
                policy,
            )? {
                Classification::Decided(relation) => relation,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match relation {
                BezierLineContactRelation::ControlHullDisjoint { .. }
                | BezierLineContactRelation::NoContact => {}
                BezierLineContactRelation::OnSupportingLine => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                BezierLineContactRelation::Contacts { contacts } => {
                    let sole_crossing_contact = contacts.len() == 1
                        && contacts[0].kind() == BezierLineContactKind::Crossing;
                    for contact in contacts {
                        let retained = retained_curve_region_parameter_contains(
                            contact.parameter(),
                            &regular_range,
                            policy,
                        )?;
                        match retained {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => continue,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                        if let Some(origin) = skipped_origin
                            && certified_origin_parameter.is_some()
                        {
                            let certified_parameter = BezierParameter2::Exact(
                                certified_origin_parameter
                                    .expect("a certified origin parameter was selected")
                                    .clone(),
                            );
                            let is_origin = retained_parameters_equal(
                                contact.parameter(),
                                &certified_parameter,
                                policy,
                            )?;
                            match is_origin {
                                Classification::Decided(true) => {
                                    if contact.kind() != BezierLineContactKind::Crossing {
                                        return Ok(Classification::Uncertain(
                                            UncertaintyReason::Boundary,
                                        ));
                                    }
                                    if origin.fragment_index == Some(fragment_index) {
                                        if source_origin_contact_was_skipped {
                                            return Ok(Classification::Uncertain(
                                                UncertaintyReason::Boundary,
                                            ));
                                        }
                                        source_origin_contact_was_skipped = true;
                                    }
                                    continue;
                                }
                                Classification::Decided(false) => {}
                                Classification::Uncertain(reason) => {
                                    // The exact representative lies on this
                                    // transverse source ray. A complete
                                    // singleton crossing is therefore the
                                    // origin even when its independently
                                    // isolated parameter cannot be ordered
                                    // against the represented witness.
                                    if sole_crossing_contact {
                                        if origin.fragment_index == Some(fragment_index) {
                                            source_origin_contact_was_skipped = true;
                                        }
                                        continue;
                                    }
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        match parallel.supporting_line_parameter_order_on_regular_range(
                            contact.parameter(),
                            &ray.line,
                            &regular_range,
                            policy,
                        )? {
                            Classification::Decided(std::cmp::Ordering::Greater) => {
                                match retained_line_contact_winding_delta(
                                    &contact,
                                    Some(&regular_range),
                                    reversed,
                                    policy,
                                )? {
                                    Classification::Decided(delta) => winding += delta,
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                }
                            }
                            Classification::Decided(std::cmp::Ordering::Equal) => {
                                if let Some(origin) = skipped_origin
                                    && contact.kind() == BezierLineContactKind::Crossing
                                {
                                    if origin.fragment_index == Some(fragment_index) {
                                        if source_origin_contact_was_skipped {
                                            return Ok(Classification::Uncertain(
                                                UncertaintyReason::Boundary,
                                            ));
                                        }
                                        source_origin_contact_was_skipped = true;
                                    }
                                    continue;
                                }
                                return Ok(Classification::Decided(RetainedRayWinding::Boundary));
                            }
                            Classification::Decided(std::cmp::Ordering::Less) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                }
            }
            continue;
        }
        let selected_curve = match fragment {
            BezierSplitFragment2::SelectedFiber(fragment) => fragment
                .rational_curve()
                .map(|curve| BezierSubcurve2::Rational(curve.clone())),
            _ => None,
        };
        let (curve, range, reversed) = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => (curve, None, false),
            BezierSplitFragment2::RetainedBezier {
                reversed,
                start,
                end,
                source_curve: curve,
                ..
            } => (
                curve,
                Some(CurveParameterRange2::new_validated(
                    CurveParameter2::from(start.clone()),
                    CurveParameter2::from(end.clone()),
                )),
                *reversed,
            ),
            BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => (
                exact_parallel_curve
                    .as_ref()
                    .expect("exact analytic component was selected above"),
                Some(CurveParameterRange2::from_bezier_range(
                    fragment.range().clone(),
                )),
                fragment.is_reversed(),
            ),
            BezierSplitFragment2::SelectedFiber(fragment) => (
                selected_curve
                    .as_ref()
                    .expect("an analytic selected-fiber fragment was handled procedurally"),
                Some(fragment.range().clone()),
                fragment.is_reversed(),
            ),
        };
        let unit = CurveParameterRange2::unit();
        let active_range = range.as_ref().unwrap_or(&unit);
        let unit_covers_range = matches!(
            crate::bezier_split::CurveParameterDomain2::new(&unit, None)
                .contains_finite_range(active_range, policy),
            Ok(Classification::Decided(true))
        );
        if unit_covers_range
            && !subcurve_control_hull_may_be_ahead(curve, point, direction_x, direction_y, policy)
        {
            continue;
        }
        let control_hull_order = unit_covers_range
            .then(|| {
                subcurve_control_hull_strict_order(curve, point, direction_x, direction_y, policy)
            })
            .flatten();
        let certified_source_crossing = skipped_origin.and_then(|origin| {
            (origin.fragment_index == Some(fragment_index))
                .then(|| {
                    origin
                        .parameter
                        .and_then(CurveParameter2::as_bezier_parameter)
                        .and_then(BezierParameter2::scalar)
                        .map(|parameter| (parameter, origin.parameter_crossing_direction(reversed)))
                })
                .flatten()
        });
        let certified_circle_relation =
            certified_source_crossing.and_then(|(parameter, crossing_direction)| {
                let retained_circle = match curve {
                    BezierSubcurve2::RationalQuadratic(curve) => {
                        curve.retained_circular_conic().is_some()
                    }
                    BezierSubcurve2::Rational(curve) => curve.retained_circular_conic().is_some(),
                    BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => false,
                };
                (unit_covers_range && retained_circle).then(|| {
                    RationalBezier2::try_from_subcurve(curve).map(|curve| {
                        curve.relation_to_line_with_certified_crossing(
                            &ray.line,
                            parameter,
                            crossing_direction,
                            policy,
                        )
                    })
                })
            });
        let relation = match certified_circle_relation.transpose() {
            Ok(Some(relation)) => relation,
            Ok(None) if !unit_covers_range => RationalBezier2::try_from_subcurve(curve)
                .map(|curve| curve.relation_to_line_on_range(&ray.line, active_range, policy))
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            Ok(None) => subcurve_relation_to_line_with_contacts(
                curve,
                &ray.line,
                Some((direction_x, direction_y)),
                policy,
            ),
            Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        let relation = if unit_covers_range
            && active_range != &unit
            && matches!(relation, Classification::Uncertain(_))
        {
            // The unused part of the unit chart may contain a pole even
            // though this retained range is finite.
            RationalBezier2::try_from_subcurve(curve)
                .map(|curve| curve.relation_to_line_on_range(&ray.line, active_range, policy))
                .unwrap_or(relation)
        } else {
            relation
        };
        let relation = match relation {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match relation {
            BezierLineContactRelation::ControlHullDisjoint { .. }
            | BezierLineContactRelation::NoContact => {}
            BezierLineContactRelation::OnSupportingLine => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            BezierLineContactRelation::Contacts { contacts } => {
                let sole_crossing_contact =
                    contacts.len() == 1 && contacts[0].kind() == BezierLineContactKind::Crossing;
                for contact in contacts {
                    if let Some(origin) = skipped_origin
                        && origin.fragment_index == Some(fragment_index)
                    {
                        let retained_origin_parameter = || {
                            origin
                                .parameter
                                .and_then(CurveParameter2::as_bezier_parameter)
                                .map_or(
                                    Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
                                    |parameter| {
                                        retained_parameters_equal(
                                            contact.parameter(),
                                            parameter,
                                            policy,
                                        )
                                    },
                                )
                        };
                        let is_origin =
                            if let Some(line_parameter) = contact.supporting_line_parameter() {
                                let strict_order = policy.strict_predicate_pass(|| {
                                    compare_reals(line_parameter, &Real::zero(), policy)
                                });
                                if let Some(order) = strict_order {
                                    Classification::Decided(order == std::cmp::Ordering::Equal)
                                } else {
                                    match policy.strict_predicate_pass(retained_origin_parameter)? {
                                        decided @ Classification::Decided(_) => decided,
                                        uncertain @ Classification::Uncertain(_)
                                            if sole_crossing_contact =>
                                        {
                                            uncertain
                                        }
                                        Classification::Uncertain(_) => {
                                            compare_reals(line_parameter, &Real::zero(), policy)
                                                .map(|order| {
                                                    Classification::Decided(
                                                        order == std::cmp::Ordering::Equal,
                                                    )
                                                })
                                                .map_or_else(retained_origin_parameter, Ok)?
                                        }
                                    }
                                }
                            } else {
                                // A sole crossing and the certified source
                                // incidence identify the origin even when its
                                // reconstructed scalar equality is undecided.
                                match policy.strict_predicate_pass(retained_origin_parameter)? {
                                    decided @ Classification::Decided(_) => decided,
                                    uncertain @ Classification::Uncertain(_)
                                        if sole_crossing_contact =>
                                    {
                                        uncertain
                                    }
                                    Classification::Uncertain(_) => retained_origin_parameter()?,
                                }
                            };
                        match is_origin {
                            Classification::Decided(true) => {
                                if source_origin_contact_was_skipped
                                    || contact.kind() != BezierLineContactKind::Crossing
                                {
                                    return Ok(Classification::Uncertain(
                                        UncertaintyReason::Boundary,
                                    ));
                                }
                                source_origin_contact_was_skipped = true;
                                continue;
                            }
                            Classification::Decided(false) => {}
                            Classification::Uncertain(_) if sole_crossing_contact => {
                                source_origin_contact_was_skipped = true;
                                continue;
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    let ahead = if let Some(line_parameter) = contact.supporting_line_parameter() {
                        compare_reals(line_parameter, &Real::zero(), policy)
                            .map(Classification::Decided)
                            .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign))
                    } else {
                        match contact.parameter() {
                            BezierParameter2::Exact(parameter) => {
                                if let Some(order) = control_hull_order {
                                    Classification::Decided(order)
                                } else {
                                    let contact_point =
                                        match subcurve_point_at(curve, parameter.clone(), policy) {
                                            Classification::Decided(point) => point,
                                            Classification::Uncertain(reason) => {
                                                return Ok(Classification::Uncertain(reason));
                                            }
                                        };
                                    let delta_x = contact_point.x() - point.x();
                                    let delta_y = contact_point.y() - point.y();
                                    let projection = Real::dot2_refs(
                                        [&delta_x, &delta_y],
                                        [direction_x, direction_y],
                                    );
                                    compare_reals(&projection, &Real::zero(), policy)
                                        .map(Classification::Decided)
                                        .unwrap_or(Classification::Uncertain(
                                            UncertaintyReason::RealSign,
                                        ))
                                }
                            }
                            BezierParameter2::Algebraic(parameter) => {
                                algebraic_contact_order_along_ray(
                                    curve,
                                    parameter,
                                    point,
                                    direction_x,
                                    direction_y,
                                    policy,
                                )?
                            }
                        }
                    };
                    let retained = retained_curve_region_parameter_contains(
                        contact.parameter(),
                        range.as_ref().unwrap_or(&CurveParameterRange2::unit()),
                        policy,
                    )?;
                    match retained {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => continue,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                    match ahead {
                        Classification::Decided(std::cmp::Ordering::Greater) => {
                            match retained_line_contact_winding_delta(
                                &contact,
                                range.as_ref(),
                                reversed,
                                policy,
                            )? {
                                Classification::Decided(delta) => winding += delta,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        Classification::Decided(std::cmp::Ordering::Equal) => {
                            if skipped_origin.is_some()
                                && contact.kind() == BezierLineContactKind::Crossing
                            {
                                continue;
                            }
                            return Ok(Classification::Decided(RetainedRayWinding::Boundary));
                        }
                        Classification::Decided(std::cmp::Ordering::Less) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
        }
    }
    if skipped_origin.is_some_and(|origin| origin.fragment_index.is_some())
        && !source_origin_contact_was_skipped
    {
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    }
    Ok(Classification::Decided(RetainedRayWinding::Winding(
        winding,
    )))
}

fn retained_parameters_equal(
    first: &BezierParameter2,
    second: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    first
        .cmp_by_refinement(second, policy)
        .map(|order| order.map(|order| order == std::cmp::Ordering::Equal))
}

fn retained_curve_region_parameter_orders(
    parameter: &BezierParameter2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<[std::cmp::Ordering; 2]>> {
    let parameter = CurveParameter2::from(parameter.clone());
    let start = match parameter.cmp_by_refinement(range.start(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(parameter
        .cmp_by_refinement(range.end(), policy)?
        .map(|end| [start, end]))
}

fn retained_curve_region_parameter_contains(
    parameter: &BezierParameter2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    Ok(
        retained_curve_region_parameter_orders(parameter, range, policy)?.map(|[start, end]| {
            start != std::cmp::Ordering::Less && end != std::cmp::Ordering::Greater
        }),
    )
}

fn rationalize_retained_subcurve(curve: &BezierSubcurve2) -> CurveResult<RationalBezier2> {
    RationalBezier2::try_from_subcurve(curve)
}

fn native_loop_bounds(
    boundary_loop: &BezierBoundaryLoop2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let Some(first) = boundary_loop.fragments().first() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let mut bounds = match subcurve_query_bounds(first, policy) {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    for fragment in &boundary_loop.fragments()[1..] {
        let fragment_bounds = match subcurve_query_bounds(fragment, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        bounds = match bounds.union(&fragment_bounds) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
    }
    Classification::Decided(bounds)
}

fn retained_loop_query_bounds(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let mut fragments = boundary_loop.fragments().iter();
    let Some(first) = fragments.next() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let mut bounds = match retained_fragment_query_bounds(first, policy) {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    for fragment in fragments {
        let fragment_bounds = match retained_fragment_query_bounds(fragment, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        bounds = match bounds.union(&fragment_bounds) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
    }
    Classification::Decided(bounds)
}

fn retained_loops_have_pairwise_disjoint_bounds(
    boundary_loops: &[CurveRegionBoundaryLoop2],
    policy: &CurveContext,
) -> Classification<bool> {
    let mut bounds = Vec::<Aabb2>::with_capacity(boundary_loops.len());
    for boundary_loop in boundary_loops {
        let current = match retained_loop_query_bounds(boundary_loop, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-disjoint-loop-roles-blocker",
                    "loop-bounds",
                );
                return Classification::Uncertain(reason);
            }
        };
        for previous in &bounds {
            match previous.overlaps(&current, policy) {
                Classification::Decided(false) => {}
                Classification::Decided(true) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-disjoint-loop-roles-blocker",
                        "overlapping-bounds",
                    );
                    return Classification::Decided(false);
                }
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-disjoint-loop-roles-blocker",
                        "bounds-overlap",
                    );
                    return Classification::Uncertain(reason);
                }
            }
        }
        bounds.push(current);
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-disjoint-loop-roles",
        "pairwise-disjoint",
    );
    Classification::Decided(true)
}

pub(crate) fn retained_fragment_query_bounds(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
        return chord
            .conservative_local_bounds_refined(0, policy)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let range = if matches!(fragment, BezierSplitFragment2::Materialized { .. }) {
        CurveParameterRange2::unit()
    } else {
        fragment.curve_region_parameter_range()
    };
    crate::curve_support::CurveSupport2::from_fragment(fragment)
        .certified_outer_bounds(&range, 0, policy)
}

/// Conservatively rejects a retained fragment box from an exact forward ray.
///
/// A linear projection reaches its extrema at box corners. The box is
/// disjoint when every corner is strictly on one side of the supporting line,
/// or strictly behind the ray origin. Only STRICT comparisons may prune a
/// fragment; unresolved signs retain it for the complete curve predicate.
fn retained_bounds_may_intersect_forward_ray(
    bounds: &Aabb2,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
) -> bool {
    let side_x = -direction_y.clone();
    let side_y = direction_x.clone();
    let zero = Real::zero();
    let mut all_side_negative = true;
    let mut all_side_positive = true;
    let mut all_behind = true;
    for (x, y) in [
        (bounds.min().x(), bounds.min().y()),
        (bounds.min().x(), bounds.max().y()),
        (bounds.max().x(), bounds.min().y()),
        (bounds.max().x(), bounds.max().y()),
    ] {
        let delta_x = x - origin.x();
        let delta_y = y - origin.y();
        let side = Real::dot2_refs([&delta_x, &delta_y], [&side_x, &side_y]);
        let forward = Real::dot2_refs([&delta_x, &delta_y], [direction_x, direction_y]);
        match compare_reals(&side, &zero, &CurveContext::STRICT) {
            Some(std::cmp::Ordering::Less) => all_side_positive = false,
            Some(std::cmp::Ordering::Greater) => all_side_negative = false,
            Some(std::cmp::Ordering::Equal) | None => {
                all_side_negative = false;
                all_side_positive = false;
            }
        }
        if compare_reals(&forward, &zero, &CurveContext::STRICT) != Some(std::cmp::Ordering::Less) {
            all_behind = false;
        }
    }
    !(all_side_negative || all_side_positive || all_behind)
}

fn classify_point_with_ray(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    ray: &BezierRay2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    let direction_x = &ray.direction_x;
    let direction_y = &ray.direction_y;
    let mut winding = 0_i32;
    for fragment in boundary_loop.fragments() {
        if !subcurve_control_hull_may_be_ahead(fragment, point, direction_x, direction_y, policy) {
            continue;
        }
        let control_hull_order =
            subcurve_control_hull_strict_order(fragment, point, direction_x, direction_y, policy);
        let relation = match subcurve_relation_to_line_with_contacts(
            fragment,
            &ray.line,
            Some((direction_x, direction_y)),
            policy,
        ) {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match relation {
            BezierLineContactRelation::ControlHullDisjoint { .. }
            | BezierLineContactRelation::NoContact => {}
            BezierLineContactRelation::OnSupportingLine => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            BezierLineContactRelation::Contacts { contacts } => {
                for contact in contacts {
                    let ahead = if let Some(line_parameter) = contact.supporting_line_parameter() {
                        compare_reals(line_parameter, &Real::zero(), policy)
                            .map(Classification::Decided)
                            .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign))
                    } else {
                        match contact.parameter() {
                            BezierParameter2::Exact(parameter) => {
                                if let Some(order) = control_hull_order {
                                    Classification::Decided(order)
                                } else {
                                    let contact_point = match subcurve_point_at(
                                        fragment,
                                        parameter.clone(),
                                        policy,
                                    ) {
                                        Classification::Decided(point) => point,
                                        Classification::Uncertain(reason) => {
                                            return Ok(Classification::Uncertain(reason));
                                        }
                                    };
                                    let delta_x = contact_point.x() - point.x();
                                    let delta_y = contact_point.y() - point.y();
                                    let projection = Real::dot2_refs(
                                        [&delta_x, &delta_y],
                                        [direction_x, direction_y],
                                    );
                                    compare_reals(&projection, &Real::zero(), policy)
                                        .map(Classification::Decided)
                                        .unwrap_or(Classification::Uncertain(
                                            UncertaintyReason::RealSign,
                                        ))
                                }
                            }
                            BezierParameter2::Algebraic(parameter) => {
                                algebraic_contact_order_along_ray(
                                    fragment,
                                    parameter,
                                    point,
                                    direction_x,
                                    direction_y,
                                    policy,
                                )?
                            }
                        }
                    };
                    match ahead {
                        Classification::Decided(std::cmp::Ordering::Greater) => {
                            match retained_line_contact_winding_delta(
                                &contact, None, false, policy,
                            )? {
                                Classification::Decided(delta) => winding += delta,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        Classification::Decided(std::cmp::Ordering::Equal) => {
                            return Ok(Classification::Decided(ContourPointLocation::Boundary));
                        }
                        Classification::Decided(std::cmp::Ordering::Less) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
        }
    }

    Ok(Classification::Decided(winding_location(
        winding, fill_rule,
    )))
}

fn control_points_may_be_ahead<'a>(
    controls: impl IntoIterator<Item = &'a Point2>,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> bool {
    controls.into_iter().any(|control| {
        let delta_x = control.x() - origin.x();
        let delta_y = control.y() - origin.y();
        let projection = Real::dot2_refs([&delta_x, &delta_y], [direction_x, direction_y]);
        policy.strict_predicate_pass(|| real_sign(&projection, policy)) != Some(RealSign::Negative)
    })
}

fn control_points_strict_order<'a>(
    controls: impl IntoIterator<Item = &'a Point2>,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> Option<std::cmp::Ordering> {
    let mut order = None;
    for control in controls {
        let delta_x = control.x() - origin.x();
        let delta_y = control.y() - origin.y();
        let projection = Real::dot2_refs([&delta_x, &delta_y], [direction_x, direction_y]);
        let current = match policy.strict_predicate_pass(|| real_sign(&projection, policy))? {
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => return None,
            RealSign::Positive => std::cmp::Ordering::Greater,
        };
        if order.is_some_and(|order| order != current) {
            return None;
        }
        order = Some(current);
    }
    order
}

fn subcurve_control_hull_may_be_ahead(
    curve: &BezierSubcurve2,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> bool {
    match curve {
        BezierSubcurve2::Quadratic(curve) => control_points_may_be_ahead(
            curve.control_points(),
            origin,
            direction_x,
            direction_y,
            policy,
        ),
        BezierSubcurve2::Cubic(curve) => control_points_may_be_ahead(
            curve.control_points(),
            origin,
            direction_x,
            direction_y,
            policy,
        ),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.common_nonzero_weight_sign(policy).is_none()
                || control_points_may_be_ahead(
                    curve.control_points(),
                    origin,
                    direction_x,
                    direction_y,
                    policy,
                )
        }
        BezierSubcurve2::Rational(_) => true,
    }
}

fn subcurve_control_hull_strict_order(
    curve: &BezierSubcurve2,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> Option<std::cmp::Ordering> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => control_points_strict_order(
            curve.control_points(),
            origin,
            direction_x,
            direction_y,
            policy,
        ),
        BezierSubcurve2::Cubic(curve) => control_points_strict_order(
            curve.control_points(),
            origin,
            direction_x,
            direction_y,
            policy,
        ),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.common_nonzero_weight_sign(policy).and_then(|_| {
                control_points_strict_order(
                    curve.control_points(),
                    origin,
                    direction_x,
                    direction_y,
                    policy,
                )
            })
        }
        BezierSubcurve2::Rational(_) => None,
    }
}

/// Counts a contact from the strict ray side occupied inside the finite
/// source interval. A start owns its positive after-side; an end owns its
/// positive before-side. Reversal negates the contribution, not ownership.
/// One-sided endpoint contacts retain `tangent_side` even when the supporting
/// carrier cannot certify a continuation outside its domain.
fn retained_line_contact_winding_delta(
    contact: &BezierLineContact,
    range: Option<&CurveParameterRange2>,
    reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<i32>> {
    let unit = CurveParameterRange2::unit();
    let range = range.unwrap_or(&unit);
    let [start, end] =
        match retained_curve_region_parameter_orders(contact.parameter(), range, policy)? {
            Classification::Decided(orders) => orders,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    use std::cmp::Ordering::{Equal, Greater, Less};
    if start == Less || end == Greater || (start == Equal && end == Equal) {
        return Ok(Classification::Decided(0));
    }
    let at_start = start == Equal;
    let at_end = end == Equal;
    let (before_positive, after_positive) = match contact.kind() {
        BezierLineContactKind::Crossing => match contact.crossing_direction() {
            Some(BezierLineCrossingDirection::NegativeToPositive) => (false, true),
            Some(BezierLineCrossingDirection::PositiveToNegative) => (true, false),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        },
        BezierLineContactKind::Tangent if !at_start && !at_end => {
            return Ok(Classification::Decided(0));
        }
        BezierLineContactKind::Tangent => match contact.tangent_side() {
            Some(LineSide::Left) => (true, true),
            Some(LineSide::Right) => (false, false),
            Some(LineSide::On) | None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
        },
    };
    let delta = spatial_ray_winding_delta(before_positive, after_positive, at_start, at_end);
    Ok(Classification::Decided(if reversed {
        -delta
    } else {
        delta
    }))
}

/// Counts only the strict positive side inside the finite source interval.
/// Artificial cut endpoints cancel when adjacent pieces are added.
fn spatial_ray_winding_delta(
    before_positive: bool,
    after_positive: bool,
    at_start: bool,
    at_end: bool,
) -> i32 {
    i32::from(!at_end && after_positive) - i32::from(!at_start && before_positive)
}

fn winding_location(winding: i32, fill_rule: FillRule) -> ContourPointLocation {
    let inside = match fill_rule {
        FillRule::NonZero => winding != 0,
        FillRule::EvenOdd => winding.rem_euclid(2) != 0,
    };
    if inside {
        ContourPointLocation::Inside
    } else {
        ContourPointLocation::Outside
    }
}

fn algebraic_contact_order_along_ray(
    curve: &BezierSubcurve2,
    parameter: &crate::BezierAlgebraicParameter2,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    let (use_x, origin_coordinate, direction_sign) = match real_sign(direction_x, policy) {
        Some(RealSign::Positive) => (true, origin.x(), RealSign::Positive),
        Some(RealSign::Negative) => (true, origin.x(), RealSign::Negative),
        Some(RealSign::Zero) => match real_sign(direction_y, policy) {
            Some(RealSign::Positive) => (false, origin.y(), RealSign::Positive),
            Some(RealSign::Negative) => (false, origin.y(), RealSign::Negative),
            Some(RealSign::Zero) => return Err(CurveError::ZeroLengthLine),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        },
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    let image = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.point_at_algebraic_parameter(parameter, policy),
        BezierSubcurve2::Cubic(curve) => curve.point_at_algebraic_parameter(parameter, policy),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.point_at_algebraic_parameter(parameter, policy)
        }
        BezierSubcurve2::Rational(curve) => curve.point_at_algebraic_parameter(parameter, policy),
    }?;
    let ordering = match image {
        Classification::Decided(image) => {
            image.coordinate_order_to_real(use_x, origin_coordinate, policy)?
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(ordering.map(|ordering| {
        if direction_sign == RealSign::Negative {
            ordering.reverse()
        } else {
            ordering
        }
    }))
}

struct BezierRay2 {
    line: LineSeg2,
    direction_x: Real,
    direction_y: Real,
}

fn ray_candidates(point: &Point2) -> Vec<BezierRay2> {
    let one = Real::one();
    let two = Real::from(2_i8);
    let directions = [
        (-one.clone(), Real::zero()),
        (Real::zero(), -one.clone()),
        (-one.clone(), -two.clone()),
        (-two.clone(), -one.clone()),
        (-one.clone(), one.clone()),
        (one.clone(), Real::zero()),
        (Real::zero(), one.clone()),
        (one.clone(), two.clone()),
        (two, one.clone()),
        (one.clone(), -one),
    ];
    directions
        .into_iter()
        .map(|(direction_x, direction_y)| {
            let endpoint = Point2::new(point.x() + &direction_x, point.y() + &direction_y);
            BezierRay2 {
                line: LineSeg2::try_new(point.clone(), endpoint)
                    .expect("fixed exact ray directions are nonzero"),
                direction_x,
                direction_y,
            }
        })
        .collect()
}

/// Returns a conservative outer box for point-query rejection.
///
/// Tight extrema are unnecessary here: polynomial control hulls contain their
/// entire curves, as do rational control hulls after a common nonzero weight
/// sign is certified.
fn subcurve_query_bounds(curve: &BezierSubcurve2, policy: &CurveContext) -> Classification<Aabb2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => Aabb2::from_points(curve.control_points()),
        BezierSubcurve2::Cubic(curve) => Aabb2::from_points(curve.control_points()),
        BezierSubcurve2::RationalQuadratic(curve)
            if curve.common_nonzero_weight_sign(policy).is_some() =>
        {
            Aabb2::from_points(curve.control_points())
        }
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(),
    }
}

fn subcurve_point_at(
    curve: &BezierSubcurve2,
    parameter: Real,
    policy: &CurveContext,
) -> Classification<Point2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => Classification::Decided(curve.point_at(parameter)),
        BezierSubcurve2::Cubic(curve) => Classification::Decided(curve.point_at(parameter)),
        BezierSubcurve2::RationalQuadratic(curve) => curve.point_at(parameter, policy),
        // The owning fragment supplies the domain; retained endpoints and
        // interior samples may lie outside the author's unit interval.
        BezierSubcurve2::Rational(curve) => curve.point_at_affine_classified(&parameter, policy),
    }
}

fn subcurve_contains_point(
    curve: &BezierSubcurve2,
    point: &Point2,
    policy: &CurveContext,
) -> Classification<bool> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.contains_point(point, policy),
        BezierSubcurve2::Cubic(curve) => RationalBezier2::try_new(
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 4],
        )
        .map_or(
            Classification::Uncertain(UncertaintyReason::Unsupported),
            |curve| curve.contains_point_classified(point, policy),
        ),
        BezierSubcurve2::RationalQuadratic(curve) => curve.contains_point(point, policy),
        BezierSubcurve2::Rational(curve) => curve.contains_point_classified(point, policy),
    }
}

fn subcurve_relation_to_line_with_contacts(
    curve: &BezierSubcurve2,
    line: &LineSeg2,
    direction: Option<(&Real, &Real)>,
    policy: &CurveContext,
) -> Classification<BezierLineContactRelation> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => direction.map_or_else(
            || curve.relation_to_line_with_contacts(line, policy),
            |(direction_x, direction_y)| {
                let original = curve.relation_to_line_with_contacts(line, policy);
                match original {
                    Classification::Decided(_) => original,
                    Classification::Uncertain(_) => {
                        exact_polynomial_line_contact_relation_from_direction(
                            &curve.control_points(),
                            line.start(),
                            direction_x,
                            direction_y,
                            policy,
                        )
                    }
                }
            },
        ),
        BezierSubcurve2::Cubic(curve) => direction.map_or_else(
            || curve.relation_to_line_with_contacts(line, policy),
            |(direction_x, direction_y)| {
                let original = curve.relation_to_line_with_contacts(line, policy);
                match original {
                    Classification::Decided(_) => original,
                    Classification::Uncertain(_) => {
                        exact_polynomial_line_contact_relation_from_direction(
                            &curve.control_points(),
                            line.start(),
                            direction_x,
                            direction_y,
                            policy,
                        )
                    }
                }
            },
        ),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.relation_to_line_with_contacts(line, policy)
        }
        BezierSubcurve2::Rational(curve) => {
            if curve.retained_circular_conic().is_some() {
                match curve.materialized_quadratic_representative(policy) {
                    Ok(Classification::Decided(Some(quadratic))) => {
                        return quadratic.relation_to_line_with_contacts(line, policy);
                    }
                    Ok(Classification::Decided(None)) => {}
                    Ok(Classification::Uncertain(reason)) => {
                        return Classification::Uncertain(reason);
                    }
                    Err(_) => {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    }
                }
            }
            curve.relation_to_line_with_contacts(line, policy)
        }
    }
}

impl BezierSubcurve2 {
    #[cfg(test)]
    pub(crate) fn signed_area_contribution(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Option<Real>>>> {
        match self {
            Self::Quadratic(curve) => {
                return curve.signed_area_contribution().map(certified_measurement);
            }
            Self::Cubic(curve) => {
                return curve.signed_area_contribution().map(certified_measurement);
            }
            Self::RationalQuadratic(_) | Self::Rational(_) => {}
        }
        resolve_certified_operation(policy, |attempt| self.signed_area_contribution_raw(attempt))
    }

    #[cfg(test)]
    pub(crate) fn signed_area_contribution_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        match self {
            Self::Quadratic(curve) => curve
                .signed_area_contribution()
                .map(|area| Classification::Decided(Some(area))),
            Self::Cubic(curve) => curve
                .signed_area_contribution()
                .map(|area| Classification::Decided(Some(area))),
            Self::RationalQuadratic(curve) => curve
                .signed_area_contribution()
                .map(Classification::Decided),
            Self::Rational(curve) => match curve.signed_area_contribution()? {
                Some(area) => Ok(Classification::Decided(Some(area))),
                None => rational_line_signed_area_contribution(curve, policy),
            },
        }
    }

    #[cfg(test)]
    pub(crate) fn area_moments_contribution(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Option<BezierAreaMoments2>>>> {
        match self {
            Self::Quadratic(curve) => {
                return curve.area_moments_contribution().map(certified_measurement);
            }
            Self::Cubic(curve) => {
                return curve.area_moments_contribution().map(certified_measurement);
            }
            Self::RationalQuadratic(_) | Self::Rational(_) => {}
        }
        resolve_certified_operation(policy, |attempt| {
            self.area_moments_contribution_raw(attempt)
        })
    }

    pub(crate) fn area_moments_contribution_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAreaMoments2>>> {
        match self {
            Self::Quadratic(curve) => curve
                .area_moments_contribution()
                .map(|moments| Classification::Decided(Some(moments))),
            Self::Cubic(curve) => curve
                .area_moments_contribution()
                .map(|moments| Classification::Decided(Some(moments))),
            Self::RationalQuadratic(curve) => match curve.area_moments_contribution()? {
                Some(moments) => Ok(Classification::Decided(Some(moments))),
                None => rational_line_area_moments_contribution(self, policy),
            },
            Self::Rational(curve) => match curve.area_moments_contribution()? {
                Some(moments) => Ok(Classification::Decided(Some(moments))),
                None => rational_line_area_moments_contribution(self, policy),
            },
        }
    }

    fn signed_area_contribution_with_cache(
        &self,
        policy: &CurveContext,
        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Classification<Option<Real>>> {
        match self {
            Self::Quadratic(curve) => curve
                .signed_area_contribution()
                .map(|area| Classification::Decided(Some(area))),
            Self::Cubic(curve) => curve
                .signed_area_contribution()
                .map(|area| Classification::Decided(Some(area))),
            Self::RationalQuadratic(curve) => curve
                .signed_area_contribution_with_cache(rational_quadratic_cache)
                .map(Classification::Decided),
            Self::Rational(curve) => match curve.signed_area_contribution()? {
                Some(area) => Ok(Classification::Decided(Some(area))),
                None => rational_line_signed_area_contribution(curve, policy),
            },
        }
    }
}

fn rational_line_signed_area_contribution(
    curve: &RationalBezier2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Real>>> {
    let Ok(line) = LineSeg2::try_new(curve.start().clone(), curve.end().clone()) else {
        return Ok(Classification::Decided(None));
    };
    match curve.relation_to_line_with_contacts(&line, policy) {
        Classification::Decided(BezierLineContactRelation::OnSupportingLine) => {}
        Classification::Decided(_) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let twice_area = curve.start().x() * curve.end().y() - curve.start().y() * curve.end().x();
    Ok(Classification::Decided(Some(
        (twice_area / Real::from(2_i8))?,
    )))
}

fn rational_line_area_moments_contribution(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierAreaMoments2>>> {
    let (start, end) = curve.endpoints();
    let Ok(line) = LineSeg2::try_new(start, end) else {
        return Ok(Classification::Decided(None));
    };
    match subcurve_relation_to_line_with_contacts(curve, &line, None, policy) {
        Classification::Decided(BezierLineContactRelation::OnSupportingLine) => {}
        Classification::Decided(_) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    BezierAreaMoments2::line_contribution(line.start(), line.end())
        .map(|moments| Classification::Decided(Some(moments)))
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod single_loop_corner_publication_tests {
    use super::*;

    #[test]
    fn selected_circle_corner_candidates_publish_normalized_single_loops() {
        let p = Point2::from_values;
        let path = CurvePath2::try_new(vec![
            LineSeg2::try_new(p(2, 0), p(4, 0)).unwrap().into(),
            crate::QuadraticBezier2::new(p(4, 0), p(3, 4), p(2, 0)).into(),
        ])
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = CurveRegion2::try_from_boundary_paths(
                std::slice::from_ref(&path),
                crate::FillRule::EvenOdd,
                &policy,
            )
            .unwrap()
            .into_value();
            assert_eq!(source.len(), 1);
            let Classification::Decided(paths) =
                source.boundary_paths(&policy).unwrap().into_value()
            else {
                panic!("the cap has an exact connected boundary");
            };
            let corner = paths[0]
                .curves()
                .iter()
                .position(|curve| curve.start().coordinates() == Some(&p(4, 0)))
                .unwrap();
            let outcome = source
                .fillet_loop_vertex(
                    0,
                    corner,
                    &crate::CurveFillet2::new((Real::one() / Real::from(2)).unwrap()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap();
            assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
            let candidates = {
                let solutions = outcome.value;
                let candidates = solutions.into_solutions();
                assert!(candidates.len() > 1, "expected multiple isolated fillets");
                candidates
            };
            assert_eq!(candidates.len(), 2);
            for candidate in candidates {
                assert!(
                    candidate
                        .boundary_loops()
                        .iter()
                        .flat_map(|boundary| boundary.fragments())
                        .any(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        ))
                );
                assert!(
                    candidate.has_regularized_filled_left_topology(&policy),
                    "corner contact connectivity does not certify a simple or regularized loop"
                );
                assert_eq!(
                    candidate.regularized_region_raw(&policy).unwrap(),
                    candidate
                );
                assert_eq!(
                    candidate.classify_point_raw(&p(10, 10), &policy).unwrap(),
                    Classification::Decided(crate::RegionPointLocation::Outside)
                );
            }
        }
    }
}

#[cfg(test)]
mod retained_point_classification_tests {
    use super::*;

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("fixture predicate: {reason:?}"),
        }
    }

    fn displaced_point(point: &Point2, policy: &CurveContext) -> CurvePoint2 {
        // Direction (3,4) gives the exact left unit normal (-4/5,3/5).
        let start = Point2::new(point.x() + q(4, 5), point.y() - q(3, 5));
        let end = Point2::new(start.x() + Real::from(3), start.y() + Real::from(4));
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(start.into(), end.into(), policy).unwrap(),
        );
        let (image, _) = crate::BezierAlgebraicChordParallelPoint2::new_pair(
            chord,
            Real::one(),
            Real::zero(),
            Real::zero(),
            policy,
        );
        image.into()
    }

    fn image(point: &Point2, kind: usize, policy: &CurveContext) -> CurvePoint2 {
        match kind {
            0 => displaced_point(point, policy),
            1 => {
                let start = Point2::new(point.x() + q(4, 5), point.y() - q(3, 5));
                let end = Point2::new(start.x() + Real::from(3), start.y() + Real::from(4));
                let parallel = RationalBezier2::try_new(vec![start, end], vec![Real::one(); 2])
                    .unwrap()
                    .parallel_left(Real::one())
                    .unwrap();
                crate::BezierAnalyticParallelPoint2::new(
                    parallel,
                    BezierParameter2::Exact(Real::zero()),
                    policy,
                )
                .into()
            }
            2 => {
                let preimage = Point2::new(point.x() - Real::from(2), point.y() + Real::from(3));
                let translation = crate::Similarity2::try_from_real_affine(
                    Real::one(),
                    Real::zero(),
                    Real::zero(),
                    Real::one(),
                    Real::from(2),
                    Real::from(-3),
                )
                .unwrap();
                crate::bezier_offset::BezierSimilarityPoint2::new(
                    displaced_point(&preimage, policy),
                    translation,
                    policy,
                )
                .into()
            }
            3 => CurvePoint2::from_endpoint(
                std::sync::Arc::new(BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                        point.clone(),
                        Point2::new(point.x() + Real::one(), point.y().clone()),
                        Point2::new(point.x() + Real::from(2), point.y().clone()),
                    )),
                }),
                true,
            ),
            _ => unreachable!(),
        }
    }

    fn check(kind: usize) {
        let coordinates = [
            (-1, 5),
            (1, 1),
            (3, 3),
            (5, 5),
            (9, 9),
            (11, 5),
            (0, 5),
            (2, 5),
            (4, 5),
            (6, 5),
            (8, 5),
            (10, 5),
            (0, 0),
            (2, 2),
            (4, 4),
        ];
        let mut checked = 0;
        let mut failures = Vec::new();
        for (policy_index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512]
            .into_iter()
            .enumerate()
        {
            let images = coordinates.map(|(x, y)| image(&Point2::from_values(x, y), kind, &policy));
            assert!(images.iter().all(|point| point.coordinates().is_none()));
            for (rectangle_index, (low, high)) in [(0, 10), (2, 8), (4, 6)].into_iter().enumerate()
            {
                let vertices = [(low, low), (high, low), (high, high), (low, high)]
                    .map(|(x, y)| Point2::from_values(x, y));
                let contour = Contour2::try_new(
                    (0..4)
                        .map(|i| {
                            Segment2::Line(
                                LineSeg2::try_new(
                                    vertices[i].clone(),
                                    vertices[(i + 1) % 4].clone(),
                                )
                                .unwrap(),
                            )
                        })
                        .collect(),
                )
                .unwrap();
                let region =
                    CurveRegion2::try_from_native_material_contours(vec![contour], &policy)
                        .unwrap()
                        .value;
                assert_eq!(region.boundary_loops().len(), 1);
                let mut expected_region = Vec::new();
                for (point_index, ((x, y), point)) in coordinates.iter().zip(&images).enumerate() {
                    let expected = if *x < low || *x > high || *y < low || *y > high {
                        ContourPointLocation::Outside
                    } else if *x == low || *x == high || *y == low || *y == high {
                        ContourPointLocation::Boundary
                    } else {
                        ContourPointLocation::Inside
                    };
                    let location = match expected {
                        ContourPointLocation::Inside => RegionPointLocation::Inside,
                        ContourPointLocation::Outside => RegionPointLocation::Outside,
                        ContourPointLocation::Boundary => RegionPointLocation::Boundary,
                    };
                    expected_region.push(Classification::Decided(location));
                    let public = region.classify_point(point, &policy).unwrap();
                    assert_eq!(public.certainty, CurveCertainty::Certified);
                    assert_eq!(public.value, Classification::Decided(location));
                    let actual =
                        classify_point_evidence_against_retained_loop(&region, 0, point, &policy);
                    if !matches!(actual, Ok(Classification::Decided(location)) if location == expected)
                    {
                        let code = match actual {
                            Ok(Classification::Decided(_)) => 0,
                            Ok(Classification::Uncertain(_)) => 1,
                            Err(_) => 2,
                        };
                        failures.push((policy_index, rectangle_index, point_index, code));
                    }
                    checked += 1;
                }
                let batch = region.classify_points(&images, &policy).unwrap();
                assert_eq!(batch.certainty, CurveCertainty::Certified);
                assert_eq!(batch.value, expected_region);
                let mixed: Vec<_> = coordinates
                    .iter()
                    .zip(&images)
                    .enumerate()
                    .map(|(index, ((x, y), point))| {
                        if index % 2 == 0 {
                            Point2::from_values(*x, *y).into()
                        } else {
                            point.clone()
                        }
                    })
                    .collect();
                let mixed = region.classify_points(&mixed, &policy).unwrap();
                assert_eq!(mixed.certainty, CurveCertainty::Certified);
                assert_eq!(mixed.value, expected_region);
            }
        }
        eprintln!(
            "retained point kind={kind} checks={checked} failures={}",
            failures.len()
        );
        assert!(
            failures.is_empty(),
            "bounded point classification failures: {failures:?}"
        );
    }

    #[test]
    fn chord_normal_points_classify_on_exact_rectangle_oracles() {
        check(0);
    }
    #[test]
    fn analytic_parallel_points_classify_on_exact_rectangle_oracles() {
        check(1);
    }
    #[test]
    fn similarity_points_classify_on_exact_rectangle_oracles() {
        check(2);
    }
    #[test]
    fn lazy_endpoint_points_classify_on_exact_rectangle_oracles() {
        check(3);
    }

    fn rectangle_path(low: i32, high: i32) -> CurvePath2 {
        let vertices = [(low, low), (high, low), (high, high), (low, high)]
            .map(|(x, y)| Point2::from_values(x, y));
        CurvePath2::try_new(
            (0..4)
                .map(|i| {
                    LineSeg2::try_new(vertices[i].clone(), vertices[(i + 1) % 4].clone())
                        .unwrap()
                        .into()
                })
                .collect(),
        )
        .unwrap()
    }

    #[test]
    fn public_point_queries_preserve_nested_islands_holes_and_empty_regions() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                &[
                    rectangle_path(0, 10),
                    rectangle_path(2, 8),
                    rectangle_path(4, 6),
                ],
                &[
                    CurveRegionLoopRole::Material,
                    CurveRegionLoopRole::Hole,
                    CurveRegionLoopRole::Material,
                ],
                &[FillRule::NonZero; 3],
                &policy,
            )
            .unwrap()
            .value;
            assert_eq!(region.boundary_loops().len(), 3);
            for kind in 0..4 {
                let cases = [
                    ((-1, -1), RegionPointLocation::Outside),
                    ((1, 1), RegionPointLocation::Inside),
                    ((3, 3), RegionPointLocation::Outside),
                    ((5, 5), RegionPointLocation::Inside),
                    ((0, 5), RegionPointLocation::Boundary),
                    ((2, 5), RegionPointLocation::Boundary),
                    ((4, 5), RegionPointLocation::Boundary),
                ];
                let points: Vec<_> = cases
                    .iter()
                    .map(|((x, y), _)| image(&Point2::from_values(*x, *y), kind, &policy))
                    .collect();
                let expected: Vec<_> = cases
                    .iter()
                    .map(|(_, location)| Classification::Decided(*location))
                    .collect();
                let batch = region.classify_points(&points, &policy).unwrap();
                assert_eq!(batch.certainty, CurveCertainty::Certified);
                assert_eq!(batch.value, expected);
                for (point, expected) in points.iter().zip(&expected) {
                    let result = region.classify_point(point, &policy).unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert_eq!(&result.value, expected);
                }
                let empty = CurveRegion2::empty()
                    .classify_points(&points, &policy)
                    .unwrap();
                assert_eq!(empty.certainty, CurveCertainty::Certified);
                assert!(
                    empty
                        .value
                        .iter()
                        .all(|v| *v == Classification::Decided(RegionPointLocation::Outside))
                );
            }
        }
    }

    #[test]
    fn generated_point_queries_preserve_the_boundary_of_a_retraced_path() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let line =
                LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(2, 0)).unwrap();
            let path =
                CurvePath2::try_new(vec![line.clone().into(), line.reversed().into()]).unwrap();
            let filled = CurveRegion2::try_from_boundary_paths(
                std::slice::from_ref(&path),
                crate::FillRule::EvenOdd,
                &policy,
            )
            .unwrap();
            assert_eq!(filled.certainty, CurveCertainty::Certified);
            assert!(filled.value.is_empty());
            for kind in 0..4 {
                for (coordinates, expected) in [
                    ((1, 0), ContourPointLocation::Boundary),
                    ((0, 0), ContourPointLocation::Boundary),
                    ((1, 1), ContourPointLocation::Outside),
                ] {
                    let point = image(
                        &Point2::from_values(coordinates.0, coordinates.1),
                        kind,
                        &policy,
                    );
                    let result = path.classify_point(&point, &policy).unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert_eq!(result.value, Classification::Decided(expected));
                }
            }
        }
    }

    #[test]
    fn generated_point_queries_reenter_an_exact_parabolic_offset() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // The cap is x^2 <= y <= 1, -1 <= x <= 1. Its outward
            // quarter-unit offset meets the y axis at -1/4 and 5/4.
            let lower = QuadraticBezier2::new(
                Point2::from_values(-1, 1),
                Point2::from_values(0, -1),
                Point2::from_values(1, 1),
            );
            let top =
                LineSeg2::try_new(Point2::from_values(1, 1), Point2::from_values(-1, 1)).unwrap();
            let path = CurvePath2::try_new(vec![lower.into(), top.into()]).unwrap();
            let cap =
                CurveRegion2::try_from_boundary_paths(&[path], crate::FillRule::EvenOdd, &policy)
                    .unwrap()
                    .value;
            let offset = cap
                .offset(q(1, 4), &OffsetCornerStyle2::Round, &policy)
                .unwrap();
            assert_eq!(offset.certainty, CurveCertainty::Certified);
            let cases = [
                (q(-1, 1), RegionPointLocation::Outside),
                (q(-1, 4), RegionPointLocation::Boundary),
                (q(1, 2), RegionPointLocation::Inside),
                (q(5, 4), RegionPointLocation::Boundary),
                (q(2, 1), RegionPointLocation::Outside),
            ];
            let points: Vec<_> = cases
                .iter()
                .map(|(y, _)| displaced_point(&Point2::new(Real::zero(), y.clone()), &policy))
                .collect();
            let expected: Vec<_> = cases
                .iter()
                .map(|(_, location)| Classification::Decided(*location))
                .collect();
            let batch = offset.value.classify_points(&points, &policy).unwrap();
            assert_eq!(batch.certainty, CurveCertainty::Certified);
            assert_eq!(batch.value, expected);
            for (point, expected) in points.iter().zip(expected) {
                let result = offset.value.classify_point(point, &policy).unwrap();
                assert_eq!(result.certainty, CurveCertainty::Certified);
                assert_eq!(result.value, expected);
            }
        }
    }
}

#[cfg(test)]
fn certified_measurement<T>(value: T) -> CurveOutcome<Classification<Option<T>>> {
    CurveOutcome::new(
        Classification::Decided(Some(value)),
        CurveCertainty::Certified,
    )
}
