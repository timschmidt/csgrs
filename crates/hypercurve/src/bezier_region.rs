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

use crate::CurvePointData2;
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
    certified_loop_fill_rules: Option<Arc<[FillRule]>>,
    regularized_filled_left_policy: Option<CurveContext>,
    certified_regularization: OnceLock<CurveRegion2>,
    strict_materialized_connectivity_certified: bool,
    filled_side_is_left: PolicyClassificationCache<Arc<[bool]>>,
    native_boundary_loops: OnceLock<Option<Arc<[BezierBoundaryLoop2]>>>,
    native_boundary_bounds: PolicyClassificationCache<Arc<[Aabb2]>>,
    line_image_region: PolicyClassificationCache<Option<LineArcRegion2>>,
    signed_area_cache: PolicyEvaluationCache<Option<Real>>,
}

impl CurveRegionData2 {
    fn new(boundary_loops: Vec<CurveRegionBoundaryLoop2>) -> Self {
        let strict_materialized_connectivity_certified =
            retained_region_has_strict_materialized_connectivity(&boundary_loops);
        Self {
            boundary_loops,
            certified_loop_roles: None,
            certified_loop_fill_rules: None,
            regularized_filled_left_policy: None,
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
        data.regularized_filled_left_policy = Some(CurveContext::STRICT);
        data.certified_loop_roles = Some(Arc::from(Vec::new()));
        data.certified_loop_fill_rules = Some(Arc::from(Vec::new()));
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
            .field(
                "certified_loop_fill_rules",
                &self.data.certified_loop_fill_rules,
            )
            .field(
                "regularized_filled_left_policy",
                &self.data.regularized_filled_left_policy,
            )
            .finish()
    }
}

impl PartialEq for CurveRegion2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.boundary_loops == other.data.boundary_loops
                && self.data.certified_loop_roles == other.data.certified_loop_roles
                && self.data.certified_loop_fill_rules == other.data.certified_loop_fill_rules
                && self.data.regularized_filled_left_policy
                    == other.data.regularized_filled_left_policy)
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
        raw.data_mut_for_construction().certified_loop_fill_rules =
            Some(Arc::from(vec![fill_rule; paths.len()]));
    }
    raw.finish_construction(policy)
}

fn curve_region_edit_error(operation: CurveOperation2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(operation, CurveFamily2::Line, cause)
}

/// Proves that two cuts on one closed retained carrier bound a nonempty source
/// interval complementary to the edited seam neighborhood. The comparison is
/// made in the carrier's native parameter field and then reversed only when
/// traversal opposes that parameterization.
fn retained_single_fragment_corner_cuts_are_separated(
    fragment: &BezierSplitFragment2,
    previous_cut: &CornerTrimCut2,
    next_cut: &CornerTrimCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let ordering = match next_cut
        .parameter
        .cmp_by_refinement(&previous_cut.parameter, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(ordering) => ordering,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                reason,
            ));
        }
    };
    Ok(if fragment.source_is_reversed() {
        ordering == std::cmp::Ordering::Greater
    } else {
        ordering == std::cmp::Ordering::Less
    })
}

enum RetainedCuspHalfRelation2 {
    Shared { complementary: bool },
    Overlap(crate::bezier_offset::BezierAlgebraicCuspSemicirclePairOverlap2),
    EndpointContacts(Vec<crate::bezier_offset::BezierAlgebraicCuspSemicirclePairContact2>),
}

enum RetainedCuspRunCandidateCut2 {
    Outside,
    Owned {
        parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
        placement: CornerPlacement2,
    },
    Uncertain(UncertaintyReason),
}

/// Proves that two selected half-circle charts lie on one supporting circle.
/// Shared storage is the allocation-free path. Independently reconstructed
/// frames reuse the authoritative circle-pair overlap publisher. Each probe is
/// STRICT-first; only an unresolved relation is retried as the operation's
/// terminal APPROXIMATE_512 decision.
fn retained_cusp_half_relation(
    source: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    target: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<RetainedCuspHalfRelation2>> {
    let chart_relation =
        policy.strict_predicate_pass(|| source.shared_frame_chart_relation(target, policy));
    let chart_relation = if matches!(chart_relation, Classification::Uncertain(_))
        && policy.permits_approximate_512()
    {
        source.shared_frame_chart_relation(target, policy)
    } else {
        chart_relation
    };
    match chart_relation {
        Classification::Decided(Some(complementary)) => {
            return Ok(Some(RetainedCuspHalfRelation2::Shared { complementary }));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::CircularArc,
                reason,
            ));
        }
    }
    let relation = policy.strict_predicate_pass(|| source.pair_intersections(target, policy));
    let relation = match relation {
        Ok(Classification::Uncertain(_)) if policy.permits_approximate_512() => {
            source.pair_intersections(target, policy)
        }
        relation => relation,
    }
    .map_err(|cause| curve_region_edit_error(operation, cause))?;
    Ok(match relation {
        Classification::Decided(
            crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(overlap),
        ) => Some(RetainedCuspHalfRelation2::Overlap(overlap)),
        Classification::Decided(
            crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(
                contacts,
            ),
        ) => Some(RetainedCuspHalfRelation2::EndpointContacts(contacts)),
        Classification::Decided(
            crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts
            | crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                ..
            },
        ) => None,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::CircularArc,
                reason,
            ));
        }
    })
}

fn retained_cusp_run_candidate_cut(
    source_semicircle: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    parameter: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    point: &crate::CurvePoint2,
    candidate: &crate::BezierAlgebraicCuspSemicircleFragment2,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<RetainedCuspRunCandidateCut2> {
    let Some(relation) =
        retained_cusp_half_relation(source_semicircle, candidate.semicircle(), operation, policy)?
    else {
        return Ok(RetainedCuspRunCandidateCut2::Outside);
    };
    let candidate_parameter = match relation {
        RetainedCuspHalfRelation2::Shared {
            complementary: false,
        } => parameter.clone(),
        RetainedCuspHalfRelation2::Shared {
            complementary: true,
        } => {
            return Ok(RetainedCuspRunCandidateCut2::Outside);
        }
        RetainedCuspHalfRelation2::Overlap(overlap) => overlap.map_parameter(parameter, true),
        RetainedCuspHalfRelation2::EndpointContacts(contacts) => {
            use crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::{
                End, Interior, Start,
            };
            use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::Exterior;
            let full_source = crate::BezierAlgebraicCuspSemicircleFragment2::full(
                source_semicircle.clone(),
                policy,
            );
            // Endpoint-only coincident halves cannot own a point certified in
            // the strict interior of the source half. This avoids asking a
            // general point locator to rediscover a known non-endpoint.
            match full_source
                .contains_parameter(parameter, false, false, policy)
                .map_err(|cause| curve_region_edit_error(operation, cause))?
            {
                Classification::Decided(true) => {
                    return Ok(RetainedCuspRunCandidateCut2::Outside);
                }
                Classification::Decided(false) | Classification::Uncertain(_) => {}
            }
            let source_location = match full_source
                .certified_incident_point_evidence_location(parameter, point, policy)
                .map_err(|cause| curve_region_edit_error(operation, cause))?
            {
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::Start,
                ) => Start,
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::End,
                ) => End,
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::Interior
                    | Exterior,
                ) => return Ok(RetainedCuspRunCandidateCut2::Outside),
                Classification::Uncertain(reason) => {
                    return Ok(RetainedCuspRunCandidateCut2::Uncertain(reason));
                }
            };
            let Some(target_location) = contacts.iter().find_map(|contact| {
                (contact.first_location == source_location).then_some(contact.second_location)
            }) else {
                return Err(ExactCurveError::invalid(
                    operation,
                    CurveFamily2::CircularArc,
                    CurveError::Topology(
                        "a coincident selected-circle endpoint lost its paired endpoint".into(),
                    ),
                ));
            };
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                match target_location {
                    Start => Real::zero(),
                    End => Real::one(),
                    Interior => {
                        return Err(ExactCurveError::invalid(
                            operation,
                            CurveFamily2::CircularArc,
                            CurveError::Topology(
                                "a coincident endpoint contact mapped to an interior parameter"
                                    .into(),
                            ),
                        ));
                    }
                },
            )
        }
    };
    use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::{
        End, Exterior, Interior, Start,
    };
    // The retained half relation certifies that the mapped parameter and
    // point lie on this candidate's supporting circle. Classify its authored
    // domain with the endpoint chord before constructing an independent dense
    // angular comparison; the latter is needed only if the structural point
    // evidence cannot decide.
    let incident_location = candidate
        .certified_incident_point_evidence_location(&candidate_parameter, point, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?;
    let placement = match incident_location {
        Classification::Decided(Interior) => Some(CornerPlacement2::Trim),
        Classification::Decided(End) if previous => Some(CornerPlacement2::Corner),
        Classification::Decided(Start) if !previous => Some(CornerPlacement2::Corner),
        Classification::Decided(Start | End | Exterior) => None,
        Classification::Uncertain(location_reason) => match candidate
            .contains_parameter(&candidate_parameter, false, false, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(true) => Some(CornerPlacement2::Trim),
            Classification::Decided(false) => {
                let include_start = previous == candidate.is_reversed();
                match candidate
                    .contains_parameter(&candidate_parameter, include_start, !include_start, policy)
                    .map_err(|cause| curve_region_edit_error(operation, cause))?
                {
                    Classification::Decided(contains) => {
                        contains.then_some(CornerPlacement2::Corner)
                    }
                    Classification::Uncertain(_) => {
                        return Ok(RetainedCuspRunCandidateCut2::Uncertain(location_reason));
                    }
                }
            }
            Classification::Uncertain(_) => {
                return Ok(RetainedCuspRunCandidateCut2::Uncertain(location_reason));
            }
        },
    };
    Ok(
        placement.map_or(RetainedCuspRunCandidateCut2::Outside, |placement| {
            RetainedCuspRunCandidateCut2::Owned {
                parameter: candidate_parameter,
                placement,
            }
        }),
    )
}

fn retained_cusp_smooth_run_neighbor(
    boundary: &CurveCornerChain2<'_>,
    index: usize,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<usize>> {
    let fragments = boundary.fragments();
    if fragments.len() <= 1 {
        return Ok(None);
    }
    let Some(next_index) = boundary.neighbor(index, previous) else {
        return Ok(None);
    };
    let (
        BezierSplitFragment2::AlgebraicCuspSemicircle(source),
        BezierSplitFragment2::AlgebraicCuspSemicircle(next),
    ) = (&fragments[index], &fragments[next_index])
    else {
        return Ok(None);
    };
    source
        .validate_policy(policy)
        .and_then(|()| next.validate_policy(policy))
        .map_err(|cause| curve_region_edit_error(operation, cause))?;
    if (source.semicircle().is_clockwise() ^ source.is_reversed())
        != (next.semicircle().is_clockwise() ^ next.is_reversed())
    {
        return Ok(None);
    }
    let (source_start, next_start) = if previous {
        (true, false)
    } else {
        (false, true)
    };
    // A selected-radial fillet diameter and the concentric source parameter
    // that authored its frame are tangent but cannot belong to one supporting
    // circle: the former is centered at a nonzero-radius point of the latter.
    // Reject that structurally distinct adjacency before asking the general
    // circle-pair kernel to rediscover a high-multiplicity tangent contact.
    match source
        .endpoint_selected_radial_source_tangent_relation(source_start, next, next_start, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(Some(_)) => return Ok(None),
        Classification::Decided(None) | Classification::Uncertain(_) => {}
    }
    let tangent_relation = policy.strict_predicate_pass(|| {
        exact_selected_circle_pair_tangent_cross_and_dot(
            source,
            source,
            source_start,
            next,
            next,
            next_start,
            policy,
        )
    });
    match tangent_relation {
        Classification::Decided((RealSign::Positive | RealSign::Negative, _))
        | Classification::Decided((RealSign::Zero, Some(RealSign::Negative))) => return Ok(None),
        Classification::Decided((RealSign::Zero, Some(RealSign::Zero))) => {
            return Err(ExactCurveError::invalid(
                operation,
                CurveFamily2::CircularArc,
                CurveError::Topology(
                    "nonzero selected-circle endpoint tangents had zero cross and dot".into(),
                ),
            ));
        }
        Classification::Decided((RealSign::Zero, Some(RealSign::Positive) | None))
        | Classification::Uncertain(_) => {}
    }
    retained_cusp_half_relation(source.semicircle(), next.semicircle(), operation, policy)
        .map(|relation| relation.map(|_| next_index))
}

/// Chooses the ancestral parameter frame for one incident smooth run. When a
/// seam endpoint was authored by a coincident-circle overlap map, solving in
/// its source chart preserves that exact evidence for later endpoint
/// ownership. The geometric carrier solve still runs only once.
fn retained_cusp_smooth_run_authority(
    boundary: &CurveCornerChain2<'_>,
    index: usize,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<usize>> {
    let Some(mut neighbor_index) =
        retained_cusp_smooth_run_neighbor(boundary, index, previous, operation, policy)?
    else {
        return Ok(None);
    };
    let mut authority_index = index;
    for _ in 1..boundary.fragments().len() {
        let (
            BezierSplitFragment2::AlgebraicCuspSemicircle(authority),
            BezierSplitFragment2::AlgebraicCuspSemicircle(neighbor),
        ) = (
            &boundary.fragments()[authority_index],
            &boundary.fragments()[neighbor_index],
        )
        else {
            unreachable!("a selected-circle smooth-run neighbor was already certified")
        };
        let source_is_authority = if previous {
            authority.endpoint_pair_overlap_source(true, neighbor, false)
        } else {
            authority.endpoint_pair_overlap_source(false, neighbor, true)
        };
        if source_is_authority != Some(false) {
            break;
        }
        authority_index = neighbor_index;
        let Some(next_index) = retained_cusp_smooth_run_neighbor(
            boundary,
            authority_index,
            previous,
            operation,
            policy,
        )?
        else {
            break;
        };
        if next_index == index {
            break;
        }
        neighbor_index = next_index;
    }
    Ok(Some(authority_index))
}

/// Rebinds one full-support selected-circle contact to the exact fragment in
/// the contiguous smooth run that owns it. This turns a representation-chart
/// extension into an authored trim without admitting a true curve extension.
fn rebind_retained_cusp_run_cut(
    boundary: &CurveCornerChain2<'_>,
    immediate_index: usize,
    source_index: usize,
    previous: bool,
    search_trim: bool,
    cut: &mut CornerTrimCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<usize>> {
    if cut.placement != CornerPlacement2::Extension && !search_trim {
        return Ok(Some(immediate_index));
    }
    let BezierSplitFragment2::AlgebraicCuspSemicircle(source) = &boundary.fragments()[source_index]
    else {
        return Ok(None);
    };
    let Some(parameter) = cut.parameter.as_algebraic_cusp().cloned() else {
        return Ok(None);
    };
    let complementary = cut.parameter.is_algebraic_cusp_complement();
    let complementary_semicircle;
    let source_semicircle = if complementary {
        complementary_semicircle = source.semicircle().complementary_half();
        &complementary_semicircle
    } else {
        source.semicircle()
    };
    let mut index = immediate_index;
    if source_index == immediate_index && cut.placement == CornerPlacement2::Extension {
        let Some(next_index) = retained_cusp_smooth_run_neighbor(
            boundary,
            immediate_index,
            previous,
            operation,
            policy,
        )?
        else {
            return Ok(None);
        };
        if next_index == immediate_index {
            return Ok(None);
        }
        index = next_index;
    }
    let mut uncertainty = None;
    for step in 0..boundary.fragments().len() {
        let BezierSplitFragment2::AlgebraicCuspSemicircle(candidate) = &boundary.fragments()[index]
        else {
            unreachable!("the smooth selected-circle neighbor was checked")
        };
        match retained_cusp_run_candidate_cut(
            source_semicircle,
            &parameter,
            &cut.point,
            candidate,
            previous,
            operation,
            policy,
        )? {
            RetainedCuspRunCandidateCut2::Outside => {}
            RetainedCuspRunCandidateCut2::Owned {
                parameter,
                placement,
            } => {
                cut.parameter = CurveParameter2::from_algebraic_cusp(parameter);
                cut.placement = placement;
                return Ok(Some(index));
            }
            RetainedCuspRunCandidateCut2::Uncertain(reason) => {
                uncertainty.get_or_insert(reason);
            }
        }
        if step + 1 == boundary.fragments().len() {
            break;
        }
        let Some(next_index) =
            retained_cusp_smooth_run_neighbor(boundary, index, previous, operation, policy)?
        else {
            break;
        };
        if next_index == immediate_index {
            break;
        }
        index = next_index;
    }
    match uncertainty {
        Some(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::CircularArc,
            reason,
        )),
        None => Ok(None),
    }
}

fn retained_corner_decision<T>(
    decision: Classification<T>,
    operation: CurveOperation2,
) -> ExactCurveResult<T> {
    match decision {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            reason,
        )),
    }
}

fn retained_chord_on_certified_line(
    line: &LineSeg2,
    start: CurvePoint2,
    end: CurvePoint2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<BezierSplitFragment2> {
    let support = retained_algebraic_line_support(line, operation, policy)?;
    match support
        .chord_between_certified_support_points(start, end, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(Some(chord)) => Ok(BezierSplitFragment2::AlgebraicChord(chord)),
        Classification::Decided(None) => Err(curve_region_edit_error(
            operation,
            CurveError::Topology("a retained line trim collapsed to one point".into()),
        )),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Line,
            reason,
        )),
    }
}

fn retained_algebraic_line_support(
    line: &LineSeg2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<crate::BezierAlgebraicChord2> {
    match crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
        CurvePoint2::from(line.start().clone()),
        CurvePoint2::from(line.end().clone()),
        policy,
    )
    .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(support) => Ok(support),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Line,
            reason,
        )),
    }
}

fn retained_selected_corner_parameter_is_in_native_chart(
    parameter: &CurveParameter2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    if parameter.as_bezier_parameter().is_none() && !parameter.is_retained_scalar() {
        return Ok(false);
    }
    let compare = |boundary: CurveParameter2| {
        retained_corner_decision(
            policy
                .strict_predicate_pass(|| parameter.cmp_by_refinement(&boundary, policy))
                .map_err(|cause| curve_region_edit_error(operation, cause))?,
            operation,
        )
    };
    Ok(
        !compare(CurveParameter2::from(BezierParameter2::Exact(Real::zero())))?.is_lt()
            && !compare(CurveParameter2::from(BezierParameter2::Exact(Real::one())))?.is_gt(),
    )
}

/// Rational cuts can use native control-net subdivision. Other scalar cuts
/// retain their source relation instead of embedding selected fields in a new
/// control net, whether the operation keeps one endpoint or cuts both ends.
fn corner_parameter_needs_retained_source(parameter: &CurveParameter2) -> bool {
    parameter.is_retained_scalar()
        || parameter.as_bezier_parameter().is_some_and(|parameter| {
            parameter
                .scalar()
                .is_none_or(|value| value.exact_rational_ref().is_none())
        })
}

fn retained_corner_fragment_extension(
    fragment: &BezierSplitFragment2,
    parameter: CurveParameter2,
    cut_point: &CurvePoint2,
    replacement: Option<&BezierSplitFragment2>,
    keep_before_cut: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
    if matches!(fragment, BezierSplitFragment2::RetainedBezier { .. })
        && let Some(chord_parameter) = parameter.as_algebraic_chord()
    {
        let promoted = BezierSplitFragment2::AlgebraicChord(chord_parameter.chord().clone());
        return retained_corner_fragment_extension(
            &promoted,
            parameter,
            cut_point,
            replacement,
            keep_before_cut,
            operation,
            policy,
        );
    }
    if let Some(replacement) = replacement {
        let BezierSplitFragment2::Materialized { curve, .. } = replacement else {
            return Ok(vec![replacement.clone()]);
        };
        let parameter = parameter.as_bezier_parameter().cloned().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            )
        })?;
        let replacement = BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: curve.clone(),
        };
        let replacement_cut = if keep_before_cut {
            Real::one()
        } else {
            Real::zero()
        };
        if parameter.scalar() == Some(&replacement_cut) {
            return Ok(vec![replacement]);
        }
        return retained_corner_fragment_trim(
            &replacement,
            CurveParameter2::from(parameter),
            cut_point,
            None,
            keep_before_cut,
            operation,
            policy,
        )
        .map(|fragment| fragment.into_iter().collect());
    }
    if let BezierSplitFragment2::SelectedFiber(fragment) = fragment {
        if !retained_selected_corner_parameter_is_in_native_chart(&parameter, operation, policy)? {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            ));
        }
        let compare = |first: &CurveParameter2, second: &CurveParameter2| {
            retained_corner_decision(
                policy
                    .strict_predicate_pass(|| first.cmp_by_refinement(second, policy))
                    .map_err(|cause| curve_region_edit_error(operation, cause))?,
                operation,
            )
        };
        let keep_lower_parameter_range = keep_before_cut != fragment.is_reversed();
        let range = fragment.range();
        if (keep_lower_parameter_range && !compare(&parameter, range.end())?.is_gt())
            || (!keep_lower_parameter_range && !compare(&parameter, range.start())?.is_lt())
        {
            return Err(curve_region_edit_error(
                operation,
                CurveError::Topology(
                    "a selected-fiber extension cut did not extend its authored range".into(),
                ),
            ));
        }
        let (source_start_point, source_end_point) = if fragment.is_reversed() {
            (fragment.end_point(), fragment.start_point())
        } else {
            (fragment.start_point(), fragment.end_point())
        };
        let (range, start_point, end_point) = if keep_lower_parameter_range {
            (
                CurveParameterRange2::new_validated(range.start().clone(), parameter),
                source_start_point.clone(),
                cut_point.clone(),
            )
        } else {
            (
                CurveParameterRange2::new_validated(parameter, range.end().clone()),
                cut_point.clone(),
                source_end_point.clone(),
            )
        };
        let extended = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                fragment.source().clone(),
                range,
                start_point,
                end_point,
            ),
        );
        return if fragment.is_reversed() {
            extended
                .reversed()
                .map(|fragment| vec![fragment])
                .map_err(|cause| curve_region_edit_error(operation, cause))
        } else {
            Ok(vec![extended])
        };
    }
    if let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment {
        return retained_cusp_fragment_extension(
            fragment,
            parameter,
            keep_before_cut,
            operation,
            policy,
        );
    }
    if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
        let parameter = parameter.as_algebraic_chord().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            )
        })?;
        let start = chord.start_parameter();
        let end = chord.end_parameter();
        let (start, end) = if keep_before_cut {
            (&start, parameter)
        } else {
            (parameter, &end)
        };
        let retained = if parameter.chord() == chord {
            crate::BezierAlgebraicChord2::from_certified_ordered_parameter_range(
                chord, start, end, policy,
            )
        } else {
            crate::BezierAlgebraicChord2::from_ordered_parameter_range(chord, start, end, policy)
        };
        return retained
            .map(|chord| vec![BezierSplitFragment2::AlgebraicChord(chord)])
            .map_err(|cause| curve_region_edit_error(operation, cause));
    }

    let BezierSplitFragment2::Materialized {
        curve: BezierSubcurve2::Quadratic(source),
        ..
    } = fragment
    else {
        return Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        ));
    };
    if source.retained_exact_line_image().is_none() {
        return Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::QuadraticBezier,
            UncertaintyReason::Unsupported,
        ));
    }
    let start = if keep_before_cut {
        CurvePoint2::from(source.start().clone())
    } else {
        cut_point.clone()
    };
    let end = if keep_before_cut {
        cut_point.clone()
    } else {
        CurvePoint2::from(source.end().clone())
    };
    if let (CurvePoint2(CurvePointData2::Exact(start)), CurvePoint2(CurvePointData2::Exact(end))) =
        (&start, &end)
    {
        let line = LineSeg2::try_new(start.clone(), end.clone())
            .map_err(|cause| curve_region_edit_error(operation, cause))?;
        return Ok(vec![BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(line)),
        }]);
    }
    retained_chord_on_certified_line(
        source
            .retained_exact_line_image()
            .expect("the retained extension source is an exact line"),
        start,
        end,
        operation,
        policy,
    )
    .map(|fragment| vec![fragment])
}

fn retained_cusp_fragment_extension(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    parameter: CurveParameter2,
    keep_before_cut: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
    fragment
        .validate_policy(policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?;
    let complementary_cut = parameter.is_algebraic_cusp_complement();
    let cut = parameter.as_algebraic_cusp().cloned().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        )
    })?;
    let compare =
        |first: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
         second: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2| {
            first
                .cmp_by_refinement(second, policy)
                .map_err(|cause| curve_region_edit_error(operation, cause))
                .and_then(|order| match order {
                    Classification::Decided(order) => Ok(order),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        operation,
                        CurveFamily2::RationalBezier,
                        reason,
                    )),
                })
        };
    let zero = crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero());
    let one = crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one());
    let start = fragment.start_parameter().clone();
    let end = fragment.end_parameter().clone();
    let cut_side = if complementary_cut {
        None
    } else {
        let start_order = compare(&cut, &start)?;
        let end_order = compare(&cut, &end)?;
        match (start_order, end_order) {
            (std::cmp::Ordering::Less, std::cmp::Ordering::Less) => Some(false),
            (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater) => Some(true),
            _ => {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "a selected-circle extension cut did not lie outside its authored span"
                            .into(),
                    ),
                ));
            }
        }
    };
    let base = fragment.semicircle().clone();
    let complement = base.complementary_half();
    let mut result = Vec::with_capacity(3);
    let mut push = |semicircle: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
                    range_start: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
                    range_end: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
                    reversed: bool|
     -> ExactCurveResult<()> {
        match compare(&range_start, &range_end)? {
            std::cmp::Ordering::Less => result.push(BezierSplitFragment2::AlgebraicCuspSemicircle(
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    semicircle.clone(),
                    range_start,
                    range_end,
                    reversed,
                    policy,
                ),
            )),
            std::cmp::Ordering::Equal => {}
            std::cmp::Ordering::Greater => {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "a selected-circle extension produced a descending stored range".into(),
                    ),
                ));
            }
        }
        Ok(())
    };

    match (fragment.is_reversed(), keep_before_cut, cut_side) {
        (false, true, Some(true)) => push(&base, start, cut, false)?,
        (false, true, Some(false)) => {
            push(&base, start, one.clone(), false)?;
            push(&complement, zero.clone(), one.clone(), false)?;
            push(&base, zero.clone(), cut, false)?;
        }
        (false, true, None) => {
            push(&base, start, one.clone(), false)?;
            push(&complement, zero.clone(), cut, false)?;
        }
        (false, false, Some(false)) => push(&base, cut, end, false)?,
        (false, false, Some(true)) => {
            push(&base, cut, one.clone(), false)?;
            push(&complement, zero.clone(), one.clone(), false)?;
            push(&base, zero.clone(), end, false)?;
        }
        (false, false, None) => {
            push(&complement, cut, one.clone(), false)?;
            push(&base, zero.clone(), end, false)?;
        }
        (true, true, Some(false)) => push(&base, cut, end, true)?,
        (true, true, Some(true)) => {
            push(&base, zero.clone(), end, true)?;
            push(&complement, zero.clone(), one.clone(), true)?;
            push(&base, cut, one.clone(), true)?;
        }
        (true, true, None) => {
            push(&base, zero.clone(), end, true)?;
            push(&complement, cut, one.clone(), true)?;
        }
        (true, false, Some(true)) => push(&base, start, cut, true)?,
        (true, false, Some(false)) => {
            push(&base, zero.clone(), cut, true)?;
            push(&complement, zero.clone(), one.clone(), true)?;
            push(&base, start, one.clone(), true)?;
        }
        (true, false, None) => {
            push(&complement, zero, cut, true)?;
            push(&base, start, one, true)?;
        }
    }
    if result.is_empty() {
        return Err(curve_region_edit_error(
            operation,
            CurveError::Topology("a selected-circle extension collapsed to one point".into()),
        ));
    }
    Ok(result)
}

fn retained_corner_fragment_trim(
    fragment: &BezierSplitFragment2,
    parameter: CurveParameter2,
    cut_point: &CurvePoint2,
    replacement_curve: Option<&BezierSubcurve2>,
    keep_before_cut: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<BezierSplitFragment2>> {
    if matches!(fragment, BezierSplitFragment2::RetainedBezier { .. }) {
        if let Some(chord_parameter) = parameter.as_algebraic_chord() {
            let promoted = BezierSplitFragment2::AlgebraicChord(chord_parameter.chord().clone());
            return retained_corner_fragment_trim(
                &promoted,
                parameter,
                cut_point,
                replacement_curve,
                keep_before_cut,
                operation,
                policy,
            );
        }
        let promoted = promoted_endpoint_image_corner_fragment(fragment, operation)?;
        return retained_corner_fragment_trim(
            &BezierSplitFragment2::AnalyticParallel(promoted),
            parameter,
            cut_point,
            replacement_curve,
            keep_before_cut,
            operation,
            policy,
        );
    }
    if !matches!(fragment, BezierSplitFragment2::Materialized { .. }) {
        if matches!(
            fragment,
            BezierSplitFragment2::SelectedFiber(_) | BezierSplitFragment2::AnalyticParallel(_)
        ) && parameter.as_bezier_parameter().is_none()
            && !parameter.is_retained_scalar()
        {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            ));
        }
        let reversed = fragment.source_is_reversed();
        let keep_lower = keep_before_cut != reversed;
        let domain = fragment.curve_region_parameter_range();
        let retained_boundary = if keep_lower {
            domain.start()
        } else {
            domain.end()
        };
        // A cut at the outer endpoint consumes this whole incident span.
        // Compare source parameters: coincident geometric endpoints can also
        // enclose a nonempty trace on a nonlinear support.
        if retained_corner_decision(
            policy
                .strict_predicate_pass(|| parameter.same_value(retained_boundary, policy))
                .map_err(|cause| curve_region_edit_error(operation, cause))?,
            operation,
        )? {
            return Ok(None);
        }
        let source_points = match fragment {
            BezierSplitFragment2::SelectedFiber(selected) => Some(if reversed {
                [selected.end_point().clone(), selected.start_point().clone()]
            } else {
                [selected.start_point().clone(), selected.end_point().clone()]
            }),
            BezierSplitFragment2::AnalyticParallel(_) if parameter.is_retained_scalar() => {
                // The first common-scalar cut keeps the untouched endpoint as
                // a lazy source image. Later cuts reuse it from SelectedFiber,
                // so repeated restrictions do not build an endpoint chain.
                let source = Arc::new(fragment.clone());
                Some([
                    CurvePoint2::from_endpoint(Arc::clone(&source), !reversed),
                    CurvePoint2::from_endpoint(source, reversed),
                ])
            }
            _ => None,
        };
        let (range, points) = if keep_lower {
            (
                CurveParameterRange2::new_validated(domain.start().clone(), parameter),
                source_points.map(|[start, _]| [start, cut_point.clone()]),
            )
        } else {
            (
                CurveParameterRange2::new_validated(parameter, domain.end().clone()),
                source_points.map(|[_, end]| [cut_point.clone(), end]),
            )
        };
        // The corner solver proved order and finite-domain placement; the
        // check above excludes a fully consumed span before publication.
        return CurveSupport2::from_fragment(fragment)
            .restrict_certified(range, points, reversed, policy)
            .map(Some)
            .map_err(|cause| curve_region_edit_error(operation, cause));
    }
    let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
        return Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        ));
    };
    let curve = replacement_curve.unwrap_or(curve);
    if let BezierSubcurve2::Quadratic(line) = curve
        && let Some(support) = line.retained_exact_line_image()
        && (parameter.as_algebraic_chord().is_some()
            || (matches!(
                parameter.as_bezier_parameter(),
                Some(BezierParameter2::Algebraic(_))
            ) && line.retained_parallel_line_tangent_contacts().is_empty()))
    {
        let (start, end) = if keep_before_cut {
            (CurvePoint2::from(line.start().clone()), cut_point.clone())
        } else {
            (cut_point.clone(), CurvePoint2::from(line.end().clone()))
        };
        // A promoted chord parameter need not use the materialized unit
        // chart. The certified line is injective, so its point identity
        // also certifies a consumed range.
        if retained_corner_decision(
            policy.strict_predicate_pass(|| start.same_point(&end, policy)),
            operation,
        )? {
            return Ok(None);
        }
        return retained_chord_on_certified_line(support, start, end, operation, policy).map(Some);
    }
    let retained_boundary = CurveParameter2::from(if keep_before_cut {
        Real::zero()
    } else {
        Real::one()
    });
    if retained_corner_decision(
        policy
            .strict_predicate_pass(|| parameter.same_value(&retained_boundary, policy))
            .map_err(|cause| curve_region_edit_error(operation, cause))?,
        operation,
    )? {
        return Ok(None);
    }
    // Keep nonrational cuts in their source chart. Rebuilding their control
    // points hides the selected parameter relation inside new coefficients,
    // forcing subsequent intersections to reconstruct the same extension.
    if corner_parameter_needs_retained_source(&parameter) {
        let zero = CurveParameter2::from(BezierParameter2::Exact(Real::zero()));
        let one = CurveParameter2::from(BezierParameter2::Exact(Real::one()));
        for (boundary, expected) in [
            (&zero, std::cmp::Ordering::Greater),
            (&one, std::cmp::Ordering::Less),
        ] {
            let ordering = match parameter
                .cmp_by_refinement(boundary, policy)
                .map_err(|cause| curve_region_edit_error(operation, cause))?
            {
                Classification::Decided(ordering) => ordering,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        operation,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            };
            if ordering != expected {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "a selected-fiber corner cut lay outside its open materialized range"
                            .into(),
                    ),
                ));
            }
        }
        let rational = RationalBezier2::try_from_subcurve(curve)
            .map_err(|cause| curve_region_edit_error(operation, cause))?;
        let (source_start, source_end) = curve.endpoint_refs();
        let (range, start_point, end_point) = if keep_before_cut {
            (
                CurveParameterRange2::new_validated(zero, parameter),
                CurvePoint2::from(source_start.clone()),
                cut_point.clone(),
            )
        } else {
            (
                CurveParameterRange2::new_validated(parameter, one),
                cut_point.clone(),
                CurvePoint2::from(source_end.clone()),
            )
        };
        return CurveSupport2::Bezier(BezierSubcurve2::Rational(rational))
            .restrict_certified(range, Some([start_point, end_point]), false, policy)
            .map(Some)
            .map_err(|cause| curve_region_edit_error(operation, cause));
    }
    let parameter = parameter.as_bezier_parameter().cloned().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        )
    })?;
    let split = match curve
        .split_at_parameters_refined(
            &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            &[parameter],
            policy,
        )
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(split) => split,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                reason,
            ));
        }
    };
    if split.fragments().len() != 2 {
        return Err(curve_region_edit_error(
            operation,
            CurveError::Topology(
                "an interior retained corner cut did not produce two source fragments".into(),
            ),
        ));
    }
    let selected = if keep_before_cut {
        split.fragments()[0].clone()
    } else {
        split.fragments()[1].clone()
    };
    let BezierSplitFragment2::Materialized {
        start,
        end,
        curve: BezierSubcurve2::Rational(curve),
    } = selected
    else {
        return Ok(Some(selected));
    };
    Ok(Some(canonicalize_retained_corner_materialization(
        BezierSplitFragment2::Materialized {
            start,
            end,
            curve: BezierSubcurve2::Rational(curve),
        },
    )))
}

/// Publishes one cut in an existing circular cell cover. The parameter,
/// point, and optional cell endpoint are already certified by the caller.
/// Complementary cells extend the retained source; authored cells stand alone.
fn retained_circular_cut_fragments(
    extension_source: Option<&[BezierSplitFragment2]>,
    spans: &[RationalBezier2],
    span_index: usize,
    parameter: &CurveParameter2,
    point: &CurvePoint2,
    endpoint: Option<BezierEndpoint>,
    keep_before_cut: bool,
) -> Vec<BezierSplitFragment2> {
    let span = &spans[span_index];
    let materialized_span = |index: usize| BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Rational(spans[index].clone()),
    };
    let partial = match (keep_before_cut, endpoint) {
        (true, Some(BezierEndpoint::End)) | (false, Some(BezierEndpoint::Start)) => {
            Some(materialized_span(span_index))
        }
        (_, Some(_)) => None,
        (_, None) => {
            let (start, end, start_point, end_point) = if keep_before_cut {
                (
                    CurveParameter2::from(BezierParameter2::Exact(Real::zero())),
                    parameter.clone(),
                    CurvePoint2::from(span.start().clone()),
                    point.clone(),
                )
            } else {
                (
                    parameter.clone(),
                    CurveParameter2::from(BezierParameter2::Exact(Real::one())),
                    point.clone(),
                    CurvePoint2::from(span.end().clone()),
                )
            };
            Some(BezierSplitFragment2::SelectedFiber(
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    BezierSelectedFiberSource2::Rational(span.clone()),
                    CurveParameterRange2::new_validated(start, end),
                    start_point,
                    end_point,
                ),
            ))
        }
    };
    let mut fragments = Vec::with_capacity(spans.len() + extension_source.map_or(0, <[_]>::len));
    if keep_before_cut {
        fragments.extend(extension_source.into_iter().flatten().cloned());
        fragments.extend((0..span_index).map(materialized_span));
        fragments.extend(partial);
    } else {
        fragments.extend(partial);
        fragments.extend((span_index + 1..spans.len()).map(materialized_span));
        fragments.extend(extension_source.into_iter().flatten().cloned());
    }
    fragments
}

fn canonicalize_retained_corner_materialization(
    fragment: BezierSplitFragment2,
) -> BezierSplitFragment2 {
    let BezierSplitFragment2::Materialized {
        start,
        end,
        curve: BezierSubcurve2::Rational(curve),
    } = fragment
    else {
        return fragment;
    };
    BezierSplitFragment2::Materialized {
        start,
        end,
        curve: canonicalize_exact_rational_subcurve(
            BezierSubcurve2::Rational(curve),
            &CurveContext::STRICT,
        ),
    }
}

/// Retains the single source interval between two cuts on one closed carrier.
/// The next cut is the interval's traversal start and the previous cut is its
/// traversal end. Interior analytic cuts preserve their global parameter
/// authority. Exterior cuts retain one certified pole-free source interval,
/// so both point witnesses and every later query keep the same parameter chart.
fn retained_corner_fragment_between_cuts(
    fragment: &BezierSplitFragment2,
    previous_cut: &CornerTrimCut2,
    next_cut: &CornerTrimCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<BezierSplitFragment2> {
    match (
        previous_cut.replacement.as_deref(),
        next_cut.replacement.as_deref(),
    ) {
        (Some(previous), Some(next)) if previous != next => {
            return Err(curve_region_edit_error(
                operation,
                CurveError::Topology(
                    "one retained corner interval had incompatible replacement carriers".into(),
                ),
            ));
        }
        (Some(replacement), _) | (_, Some(replacement))
            if !matches!(replacement, BezierSplitFragment2::Materialized { .. }) =>
        {
            return Ok(replacement.clone());
        }
        _ => {}
    }
    if matches!(fragment, BezierSplitFragment2::RetainedBezier { .. }) {
        if let Some(chord_parameter) = previous_cut
            .parameter
            .as_algebraic_chord()
            .or_else(|| next_cut.parameter.as_algebraic_chord())
        {
            let promoted = BezierSplitFragment2::AlgebraicChord(chord_parameter.chord().clone());
            return retained_corner_fragment_between_cuts(
                &promoted,
                previous_cut,
                next_cut,
                operation,
                policy,
            );
        }
        let promoted = promoted_endpoint_image_corner_fragment(fragment, operation)?;
        return retained_corner_fragment_between_cuts(
            &BezierSplitFragment2::AnalyticParallel(promoted),
            previous_cut,
            next_cut,
            operation,
            policy,
        );
    }
    if let BezierSplitFragment2::SelectedFiber(selected) = fragment
        && (previous_cut.placement == CornerPlacement2::Extension
            || next_cut.placement == CornerPlacement2::Extension)
    {
        if previous_cut.replacement.is_none()
            && next_cut.replacement.is_none()
            && retained_selected_corner_parameter_is_in_native_chart(
                &previous_cut.parameter,
                operation,
                policy,
            )?
            && retained_selected_corner_parameter_is_in_native_chart(
                &next_cut.parameter,
                operation,
                policy,
            )?
        {
            let (lower_parameter, upper_parameter, lower_point, upper_point) =
                if selected.is_reversed() {
                    (
                        previous_cut.parameter.clone(),
                        next_cut.parameter.clone(),
                        previous_cut.point.clone(),
                        next_cut.point.clone(),
                    )
                } else {
                    (
                        next_cut.parameter.clone(),
                        previous_cut.parameter.clone(),
                        next_cut.point.clone(),
                        previous_cut.point.clone(),
                    )
                };
            let order = retained_corner_decision(
                policy
                    .strict_predicate_pass(|| {
                        lower_parameter.cmp_by_refinement(&upper_parameter, policy)
                    })
                    .map_err(|cause| curve_region_edit_error(operation, cause))?,
                operation,
            )?;
            if order != std::cmp::Ordering::Less {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "selected extension cuts did not bound one traversal interval".into(),
                    ),
                ));
            }
            return CurveSupport2::from_fragment(fragment)
                .restrict_certified(
                    CurveParameterRange2::new_validated(lower_parameter, upper_parameter),
                    Some([lower_point, upper_point]),
                    selected.is_reversed(),
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(operation, cause));
        }
        return Err(curve_region_edit_error(
            operation,
            CurveError::Topology(
                "a selected exterior interval reached reconstruction without its certified source range"
                    .into(),
            ),
        ));
    }
    if let BezierSplitFragment2::AnalyticParallel(fragment) = fragment
        && (previous_cut.placement == CornerPlacement2::Extension
            || next_cut.placement == CornerPlacement2::Extension)
    {
        let previous = previous_cut
            .parameter
            .as_bezier_parameter()
            .cloned()
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                )
            })?;
        let next = next_cut
            .parameter
            .as_bezier_parameter()
            .cloned()
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                )
            })?;
        // In traversal terms the retained complement runs from the next cut
        // to the previous cut. The same oriented pair is ascending for a
        // forward carrier and descending for a reversed carrier; `try_new`
        // normalizes storage while retaining that traversal bit.
        let range = BezierParameterRange2::new_validated(next, previous);
        let rebuilt = retained_corner_decision(
            crate::BezierParallelFragment2::try_new(fragment.parallel().clone(), range, policy)
                .map_err(|cause| curve_region_edit_error(operation, cause))?,
            operation,
        )?;
        return Ok(BezierSplitFragment2::AnalyticParallel(rebuilt));
    }
    if let BezierSplitFragment2::Materialized { curve, .. } = fragment {
        let replacement = match (
            previous_cut.replacement_curve(),
            next_cut.replacement_curve(),
        ) {
            (Some(previous), Some(next)) if previous != next => {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "one retained corner interval had incompatible canonical sources".into(),
                    ),
                ));
            }
            (Some(curve), _) | (_, Some(curve)) => Some(curve.clone()),
            (None, None) => None,
        };
        let curve = replacement.as_ref().unwrap_or(curve);
        if [&next_cut.parameter, &previous_cut.parameter]
            .into_iter()
            .any(corner_parameter_needs_retained_source)
        {
            for parameter in [&next_cut.parameter, &previous_cut.parameter] {
                if !retained_selected_corner_parameter_is_in_native_chart(
                    parameter, operation, policy,
                )? {
                    return Err(curve_region_edit_error(
                        operation,
                        CurveError::Topology(
                            "a selected corner interval left its materialized source chart".into(),
                        ),
                    ));
                }
            }
            // The corner chain already certified separation in this chart.
            // Keep both selected point witnesses on the original support,
            // just as the one-sided cut publisher does.
            return CurveSupport2::Bezier(curve.clone())
                .restrict_certified(
                    CurveParameterRange2::new_validated(
                        next_cut.parameter.clone(),
                        previous_cut.parameter.clone(),
                    ),
                    Some([next_cut.point.clone(), previous_cut.point.clone()]),
                    false,
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(operation, cause));
        }
        let previous_parameter = previous_cut
            .parameter
            .as_bezier_parameter()
            .cloned()
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                )
            })?;
        let next_parameter = next_cut
            .parameter
            .as_bezier_parameter()
            .cloned()
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                )
            })?;
        let zero = BezierParameter2::Exact(Real::zero());
        let one = BezierParameter2::Exact(Real::one());
        let next_order = match next_parameter
            .cmp_by_refinement(&zero, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    reason,
                ));
            }
        };
        let previous_order = match previous_parameter
            .cmp_by_refinement(&one, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    reason,
                ));
            }
        };
        if next_order == std::cmp::Ordering::Less || previous_order == std::cmp::Ordering::Greater {
            return Err(curve_region_edit_error(
                operation,
                CurveError::Topology(
                    "a canonical one-fragment corner interval left its replacement envelope".into(),
                ),
            ));
        }
        let mut parameters = Vec::with_capacity(2);
        let selected_index = if next_order == std::cmp::Ordering::Greater {
            parameters.push(next_parameter);
            1
        } else {
            0
        };
        if previous_order == std::cmp::Ordering::Less {
            parameters.push(previous_parameter);
        }
        let split = match curve
            .split_at_parameters_refined(
                &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                &parameters,
                policy,
            )
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(split) => split,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    reason,
                ));
            }
        };
        if split.fragments().len() != parameters.len() + 1
            || selected_index >= split.fragments().len()
        {
            return Err(curve_region_edit_error(
                operation,
                CurveError::Topology(
                    "two separated retained corner cuts did not isolate one source interval".into(),
                ),
            ));
        }
        return Ok(canonicalize_retained_corner_materialization(
            split.fragments()[selected_index].clone(),
        ));
    }

    let reversed = fragment.source_is_reversed();
    let (lower, upper) = if reversed {
        (previous_cut, next_cut)
    } else {
        (next_cut, previous_cut)
    };
    let points = match fragment {
        BezierSplitFragment2::SelectedFiber(_) => Some([lower.point.clone(), upper.point.clone()]),
        BezierSplitFragment2::AnalyticParallel(_)
            if lower.parameter.is_retained_scalar() || upper.parameter.is_retained_scalar() =>
        {
            Some([lower.point.clone(), upper.point.clone()])
        }
        _ => None,
    };
    CurveSupport2::from_fragment(fragment)
        .restrict_certified(
            CurveParameterRange2::new_validated(lower.parameter.clone(), upper.parameter.clone()),
            points,
            reversed,
            policy,
        )
        .map_err(|cause| curve_region_edit_error(operation, cause))
}

/// Keeps both cuts, their point witnesses and their original support chart.
/// The finite envelope certifies source finiteness; it never becomes a new
/// geometric carrier or a restriction on the exact cut representation.
fn retain_corner_extension_interval(
    fragment: &BezierSplitFragment2,
    previous_cut: &mut CornerTrimCut2,
    next_cut: &mut CornerTrimCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let reversed = fragment.source_is_reversed();
    let (lower, upper) = if reversed {
        (&*previous_cut, &*next_cut)
    } else {
        (&*next_cut, &*previous_cut)
    };
    let order = retained_corner_decision(
        policy
            .strict_predicate_pass(|| lower.parameter.cmp_by_refinement(&upper.parameter, policy))
            .map_err(|cause| curve_region_edit_error(operation, cause))?,
        operation,
    )?;
    if order != std::cmp::Ordering::Less {
        return Err(curve_region_edit_error(
            operation,
            CurveError::Topology(
                "retained extension cuts did not bound one traversal interval".into(),
            ),
        ));
    }
    let range =
        CurveParameterRange2::new_validated(lower.parameter.clone(), upper.parameter.clone());
    let support = CurveSupport2::from_fragment(fragment);
    let source = match &support {
        CurveSupport2::Bezier(curve) => RationalBezier2::try_from_subcurve(curve),
        CurveSupport2::Parallel(parallel) => parallel.source().to_rational_bezier(),
        CurveSupport2::Line(_) | CurveSupport2::Circle(_) => {
            return Err(ExactCurveError::blocked(
                operation,
                support.family(),
                UncertaintyReason::Unsupported,
            ));
        }
    }
    .map_err(|cause| curve_region_edit_error(operation, cause))?;
    retained_corner_decision(
        source
            .finite_discovery_envelope(&range, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?,
        operation,
    )?;
    let replacement = Arc::new(
        support
            .restrict_certified(
                range,
                Some([lower.point.clone(), upper.point.clone()]),
                reversed,
                policy,
            )
            .map_err(|cause| curve_region_edit_error(operation, cause))?,
    );
    previous_cut.replacement = Some(replacement.clone());
    next_cut.replacement = Some(replacement);
    Ok(())
}

fn curve_region_boundary_loop_from_native_material_contour(
    contour: Contour2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegionBoundaryLoop2> {
    let mut boundaries =
        CurveRegion2::try_from_native_contours_raw(vec![contour], Vec::new(), policy)?
            .into_boundary_loops();
    if boundaries.len() != 1 {
        return Err(curve_region_edit_error(
            CurveOperation2::Offset,
            CurveError::Topology("a native offset cap did not produce one boundary loop".into()),
        ));
    }
    // Promotion-local arrangement indices start at zero for every cap. They
    // are not source-curve identities and cannot be combined across caps;
    // the unified band arrangement publishes its own global provenance.
    Ok(boundaries
        .pop()
        .expect("the native offset cap has exactly one boundary loop")
        .without_arrangement_sources())
}

struct ExactOffsetSpan2 {
    fragments: Vec<BezierSplitFragment2>,
    source_end: CurvePoint2,
    offset_start: CurvePoint2,
    offset_end: CurvePoint2,
    start_tangent: Option<CurveTangent2>,
    end_tangent: Option<CurveTangent2>,
}

pub(crate) enum CurveTangent2 {
    /// Represented traversal direction. Its magnitude is arbitrary; metric
    /// constructions must request a certified unit direction explicitly.
    RepresentedDirection((Real, Real)),
    /// Traversal tangent of an analytic parallel at an algebraic parameter.
    /// `source_direction` is the nonzero orientation relative to the
    /// parallel source's homogeneous tangent numerator.
    RetainedParallel {
        parallel: BezierParallel2,
        source_parallel: BezierParallel2,
        source_range: CurveParameterRange2,
        parameter: BezierParameter2,
        /// Original compact selected-fiber scalar, when this tangent came
        /// from a selected region fragment. Keeping its one-word authority
        /// avoids re-proving equality against the promoted global root.
        selected_source_parameter:
            Option<crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2>,
        source_direction: RealSign,
    },
    AlgebraicChord(crate::BezierAlgebraicChord2),
    CircularPoint {
        point: CurvePoint2,
        circle: Arc<crate::rational_bezier::RationalQuadraticCircle2>,
        clockwise: bool,
    },
    SelectedCircularEndpoint {
        /// Source carrier before the concentric offset.  Smooth carrier
        /// switches can need its exact overlap map after the two offset
        /// endpoint images have moved into independent selected fields.
        source_fragment: crate::BezierAlgebraicCuspSemicircleFragment2,
        fragment: crate::BezierAlgebraicCuspSemicircleFragment2,
        at_start: bool,
    },
    ChordContact {
        fragment: crate::BezierAlgebraicCuspSemicircleFragment2,
        at_start: bool,
        chord: crate::BezierAlgebraicChord2,
        circle_cross_chord: RealSign,
        circle_dot_chord: Option<RealSign>,
    },
}

impl CurveTangent2 {
    /// Retains a boundary's traversal direction in the same geometric chart
    /// used by offsets. No offset construction or coordinate projection is
    /// needed to compare endpoint directions in an arrangement.
    pub(crate) fn at_boundary_endpoint(
        fragment: &BezierSplitFragment2,
        at_start: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let (parallel, range, reversed) = match fragment {
            BezierSplitFragment2::AlgebraicChord(chord) => {
                return Ok(Classification::Decided(Self::AlgebraicChord(chord.clone())));
            }
            BezierSplitFragment2::AlgebraicCuspSemicircle(circle) => {
                return Ok(
                    match selected_circle_endpoint_tangent(circle, circle, at_start, policy)? {
                        Classification::Decided(Some(tangent)) => Classification::Decided(tangent),
                        Classification::Decided(None) => {
                            Classification::Uncertain(UncertaintyReason::Unsupported)
                        }
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    },
                );
            }
            BezierSplitFragment2::Materialized { curve, .. } => {
                // A polynomial Bezier's endpoint derivative is n times its
                // first or last control difference; no power basis is needed.
                let polynomial_endpoint = |controls: &[&Point2]| {
                    let degree = Real::from(controls.len() as i64 - 1);
                    let (from, to) = if at_start {
                        (controls[0], controls[1])
                    } else {
                        (controls[controls.len() - 2], controls[controls.len() - 1])
                    };
                    crate::CurveDerivative2::new(
                        (to.x() - from.x()) * &degree,
                        (to.y() - from.y()) * &degree,
                    )
                };
                let derivative = match curve {
                    BezierSubcurve2::Quadratic(curve) => {
                        polynomial_endpoint(&curve.control_points())
                    }
                    BezierSubcurve2::Cubic(curve) => polynomial_endpoint(&curve.control_points()),
                    BezierSubcurve2::RationalQuadratic(_) | BezierSubcurve2::Rational(_) => {
                        let curve = RationalBezier2::try_from_subcurve(curve)?;
                        let parameter = if at_start { Real::zero() } else { Real::one() };
                        match curve.derivative_at_classified(&parameter, policy) {
                            Classification::Decided(derivative) => derivative,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                };
                if derivative.zero_status() != hyperreal::ZeroKnowledge::NonZero
                    && ![derivative.dx(), derivative.dy()]
                        .into_iter()
                        .any(|component| is_zero(component, policy) == Some(false))
                {
                    // Stationary endpoints require a one-sided higher-order
                    // direction. A zero derivative cannot order face branches.
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                return Ok(Classification::Decided(Self::RepresentedDirection((
                    derivative.dx().clone(),
                    derivative.dy().clone(),
                ))));
            }
            BezierSplitFragment2::RetainedBezier {
                source_curve,
                start,
                end,
                reversed,
                ..
            } => (
                RationalBezier2::try_from_subcurve(source_curve)?.parallel_left(Real::zero())?,
                CurveParameterRange2::from_bezier_range(BezierParameterRange2::new_validated(
                    start.clone(),
                    end.clone(),
                )),
                *reversed,
            ),
            BezierSplitFragment2::AnalyticParallel(fragment) => (
                fragment.parallel().clone(),
                CurveParameterRange2::from_bezier_range(fragment.range().clone()),
                fragment.is_reversed(),
            ),
            BezierSplitFragment2::SelectedFiber(fragment) => (
                fragment.parallel_carrier(),
                fragment.range().clone(),
                fragment.is_reversed(),
            ),
        };
        let range = if matches!(fragment, BezierSplitFragment2::SelectedFiber(_)) {
            let analysis = match parallel.singularity_analysis(&range, policy)? {
                Classification::Decided(analysis) => analysis,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let mut ranges = match analysis.regular_subranges(policy)? {
                Classification::Decided(ranges) => ranges,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            if at_start != reversed {
                ranges.remove(0)
            } else {
                ranges
                    .pop()
                    .expect("a regular partition retains the source range")
            }
        } else {
            range
        };
        let scale = match retained_parallel_range_scale_sign(&parallel, &range, policy)? {
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(scale) => scale,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let parameter = if at_start != reversed {
            range.start()
        } else {
            range.end()
        };
        exact_parallel_region_endpoint_tangent(
            &parallel, &parallel, &range, parameter, scale, reversed, policy,
        )
    }

    /// Orders outgoing directions around an incoming traversal with its filled
    /// face on the left. Equal directions retain their higher-order ambiguity.
    pub(crate) fn compare_filled_left_turn(
        &self,
        first: &Self,
        second: &Self,
        policy: &CurveContext,
    ) -> Classification<std::cmp::Ordering> {
        let half = |candidate| match curve_tangent_cross_sign(self, candidate, policy) {
            Classification::Decided(RealSign::Positive) => Classification::Decided((0_u8, false)),
            Classification::Decided(RealSign::Negative) => Classification::Decided((1, false)),
            Classification::Decided(RealSign::Zero) => {
                curve_tangents_are_opposite(self, candidate, policy)
                    .map(|opposite| (u8::from(opposite), true))
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        };
        let ((first_half, first_collinear), (second_half, second_collinear)) =
            match (half(first), half(second)) {
                (Classification::Decided(first), Classification::Decided(second)) => {
                    (first, second)
                }
                (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                    return Classification::Uncertain(reason);
                }
            };
        if first_half != second_half {
            return Classification::Decided(first_half.cmp(&second_half));
        }
        // In either half, a ray parallel to the reference is the last in
        // clockwise order. Reuse that certified relation instead of joining
        // the two candidate fields to rediscover their determinant sign.
        // Two such rays in the same half have the same direction.
        if first_collinear || second_collinear {
            return Classification::Decided(first_collinear.cmp(&second_collinear));
        }
        curve_tangent_cross_sign(first, second, policy).map(|sign| match sign {
            RealSign::Positive => std::cmp::Ordering::Greater,
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
        })
    }
}

fn exact_offset_tangent_is_selected_circle(tangent: &CurveTangent2) -> bool {
    matches!(
        tangent,
        CurveTangent2::SelectedCircularEndpoint { .. } | CurveTangent2::ChordContact { .. }
    )
}

/// Certifies one monotone bevel coordinate directly from incident tangent
/// component signs. Unit normalization preserves each component sign; when
/// the two signs differ, the corresponding normal-component difference has a
/// strict sign independent of either speed magnitude.
fn exact_offset_bevel_parameter_axis(
    previous: &CurveTangent2,
    next: &CurveTangent2,
    turn_sign: RealSign,
    distance_sign: RealSign,
    policy: &CurveContext,
) -> Option<(crate::Axis2, bool)> {
    let (CurveTangent2::AlgebraicChord(previous), CurveTangent2::AlgebraicChord(next)) =
        (previous, next)
    else {
        return None;
    };
    let component_difference = |first, second| match (first, second) {
        (RealSign::Negative, RealSign::Zero | RealSign::Positive)
        | (RealSign::Zero, RealSign::Positive) => Some(RealSign::Positive),
        (RealSign::Positive, RealSign::Zero | RealSign::Negative)
        | (RealSign::Zero, RealSign::Negative) => Some(RealSign::Negative),
        _ => None,
    };
    let mut resolved_signs = [(None, None); 2];
    for (axis_index, tangent_axis) in [crate::Axis2::X, crate::Axis2::Y].into_iter().enumerate() {
        let (previous_certified, next_certified) = (
            previous.certified_tangent_axis_sign(tangent_axis),
            next.certified_tangent_axis_sign(tangent_axis),
        );
        let component_sign = |chord: &crate::BezierAlgebraicChord2, certified: Option<RealSign>| {
            certified.or_else(|| match chord.tangent_axis_sign(tangent_axis, policy) {
                Ok(Classification::Decided(sign)) => Some(sign),
                Ok(Classification::Uncertain(_)) | Err(_) => None,
            })
        };
        let (Some(previous_sign), Some(next_sign)) = (
            component_sign(previous, previous_certified),
            component_sign(next, next_certified),
        ) else {
            continue;
        };
        resolved_signs[axis_index] = (Some(previous_sign), Some(next_sign));
        let difference = component_difference(previous_sign, next_sign);
        let Some(difference) = difference else {
            continue;
        };
        let (axis, normal_difference) = match tangent_axis {
            // N_y = T_x / |T|.
            crate::Axis2::X => (crate::Axis2::Y, difference),
            // N_x = -T_y / |T|.
            crate::Axis2::Y => (crate::Axis2::X, exact_sign_reverse(difference)),
        };
        let sign = exact_sign_product(distance_sign, normal_difference);
        if sign == RealSign::Zero {
            return None;
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-bevel-axis",
            "separated-tangent-components",
        );
        return Some((axis, sign == RealSign::Positive));
    }
    let [
        (Some(previous_x), Some(next_x)),
        (Some(previous_y), Some(next_y)),
    ] = resolved_signs
    else {
        return None;
    };
    if previous_x == next_x
        && previous_y == next_y
        && previous_x != RealSign::Zero
        && previous_y != RealSign::Zero
        && turn_sign != RealSign::Zero
    {
        // Both unit tangents lie in one open quadrant. That quadrant is
        // narrower than pi, so the cross sign fixes their angular order and
        // component monotonicity without comparing either normalized
        // magnitude: d(T_x)/d(theta)=-T_y.
        let difference_x = exact_sign_product(turn_sign, exact_sign_reverse(previous_y));
        let sign = exact_sign_product(distance_sign, difference_x);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-bevel-axis",
            "same-quadrant-turn",
        );
        return Some((crate::Axis2::Y, sign == RealSign::Positive));
    }
    None
}

fn retained_chord_fragment(chord: crate::BezierAlgebraicChord2) -> BezierSplitFragment2 {
    BezierSplitFragment2::AlgebraicChord(chord)
}

fn append_exact_algebraic_line_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    from: &crate::CurvePoint2,
    to: &crate::CurvePoint2,
    certified_direction: Option<BezierAlgebraicChordAxisDirection2>,
    certified_parameter_axis: Option<(crate::Axis2, bool)>,
    certified_distinct: bool,
    certified_circle_transverse_endpoints: [bool; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let endpoint_equality = if certified_direction.is_some()
        || certified_parameter_axis.is_some()
        || certified_distinct
    {
        Classification::Decided(false)
    } else {
        from.same_point(to, policy)
    };
    match endpoint_equality {
        Classification::Decided(true) => Ok(Classification::Decided(())),
        Classification::Decided(false) => {
            if let (Some(from), Some(to)) = (from.coordinates(), to.coordinates()) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-line-join",
                    "represented-endpoints",
                );
                let line = LineSeg2::try_new(from.clone(), to.clone())?;
                fragments.push(materialized_offset_fragment(BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(line),
                )));
                return Ok(Classification::Decided(()));
            }
            let chord = if let Some(direction) = certified_direction {
                crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
                    from.clone(),
                    to.clone(),
                    direction,
                    policy,
                )
            } else if let Some((axis, coordinate_increases)) = certified_parameter_axis {
                crate::BezierAlgebraicChord2::from_certified_monotone_axis_endpoints(
                    from.clone(),
                    to.clone(),
                    axis,
                    coordinate_increases,
                    policy,
                )
            } else {
                let chord = if certified_distinct {
                    crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                        from.clone(),
                        to.clone(),
                        policy,
                    )?
                } else {
                    crate::BezierAlgebraicChord2::try_new(from.clone(), to.clone(), policy)?
                };
                match chord {
                    Classification::Decided(chord) => chord,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let chord = chord
                .with_certified_circle_transverse_endpoints(certified_circle_transverse_endpoints);
            fragments.push(retained_chord_fragment(chord));
            Ok(Classification::Decided(()))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn exact_circular_algebraic_endpoint_tangent(
    curve: &RationalBezier2,
    parameter: &BezierParameter2,
    point: &CurvePoint2,
    circle: &Arc<crate::rational_bezier::RationalQuadraticCircle2>,
    clockwise: bool,
    reversed: bool,
    policy: &CurveContext,
) -> Classification<Option<CurveTangent2>> {
    match parameter {
        BezierParameter2::Exact(parameter) => curve
            .derivative_at_classified(parameter, policy)
            .map(|derivative| {
                let tangent = (derivative.dx().clone(), derivative.dy().clone());
                Some(CurveTangent2::RepresentedDirection(if reversed {
                    (-tangent.0, -tangent.1)
                } else {
                    tangent
                }))
            }),
        BezierParameter2::Algebraic(_) => {
            Classification::Decided(Some(CurveTangent2::CircularPoint {
                point: point.clone(),
                circle: Arc::clone(circle),
                clockwise: clockwise != reversed,
            }))
        }
    }
}

fn exact_offset_spans_from_algebraic_endpoint_images(
    reversed: bool,
    start: &BezierParameter2,
    end: &BezierParameter2,
    source: &BezierSubcurve2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactOffsetSpan2>>> {
    let general_offset = || {
        let parallel = retained_subcurve_parallel(source, Real::zero())?;
        let fragment = crate::BezierParallelFragment2::from_certified_range(
            parallel,
            BezierParameterRange2::new_validated(start.clone(), end.clone()),
            reversed,
        );
        exact_offset_spans_from_retained_parallel_fragment(
            RetainedParallelOffsetFragmentRef2::Analytic(&fragment),
            distance,
            policy,
        )
    };
    let BezierSubcurve2::RationalQuadratic(source_curve) = source else {
        return general_offset();
    };
    // Circular recognition selects a smaller native carrier, so it must be a
    // STRICT certificate.  An unresolved or noncircular conic simply rejoins
    // the complete analytic-parallel path below.
    let source_arc = match crate::arc_bezier::rational_quadratic_circular_arc(
        source_curve,
        &CurveContext::STRICT,
    )? {
        Classification::Decided(Some(arc)) => arc,
        Classification::Decided(None) | Classification::Uncertain(_) => return general_offset(),
    };
    let source_subcurve = BezierSubcurve2::RationalQuadratic(source_curve.clone());
    let source_rational = RationalBezier2::try_from_subcurve(&source_subcurve)?;
    let (traversal_start, traversal_end) = if reversed { (end, start) } else { (start, end) };
    let source_end = match crate::rational_bezier_general::exact_contact_point_evidence(
        &source_rational,
        traversal_end,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let carrier_distance = if reversed {
        -distance
    } else {
        distance.clone()
    };
    let radial_scale = source_arc.left_offset_radius_scale(&carrier_distance)?;
    match real_sign(&radial_scale, policy) {
        Some(RealSign::Zero) => {
            let center = CurvePoint2::from(source_arc.center().clone());
            return Ok(Classification::Decided(vec![ExactOffsetSpan2 {
                fragments: Vec::new(),
                source_end,
                offset_start: center.clone(),
                offset_end: center,
                start_tangent: None,
                end_tangent: None,
            }]));
        }
        Some(RealSign::Positive | RealSign::Negative) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let scale_point = |point: &Point2| {
        let (x, y) = point.delta_from(source_arc.center());
        source_arc
            .center()
            .translated(&x * &radial_scale, &y * &radial_scale)
    };
    let radius_squared = source_arc.radius_squared_ref() * &radial_scale * &radial_scale;
    let two = Real::from(2_i8);
    let implicit = Arc::new([
        Real::one(),
        Real::zero(),
        Real::one(),
        -(&two * source_arc.center().x()),
        -(&two * source_arc.center().y()),
        source_arc.center().x() * source_arc.center().x()
            + source_arc.center().y() * source_arc.center().y()
            - &radius_squared,
    ]);
    let offset_circle = Arc::new(crate::rational_bezier::RationalQuadraticCircle2 {
        center: source_arc.center().clone(),
        radius_squared,
        tangent_contacts: None,
    });
    let offset_curve =
        crate::RationalQuadraticBezier2::try_new_with_common_weight_sign_and_implicit_conic(
            scale_point(source_curve.start()),
            scale_point(source_curve.control()),
            scale_point(source_curve.end()),
            source_curve.start_weight().clone(),
            source_curve.control_weight().clone(),
            source_curve.end_weight().clone(),
            source_curve.common_nonzero_weight_sign(policy),
            Some(implicit),
            Some(Arc::clone(&offset_circle)),
        )?;
    let offset_subcurve = BezierSubcurve2::RationalQuadratic(offset_curve);
    let endpoint_image = |parameter: &BezierParameter2| -> CurveResult<_> {
        match parameter {
            BezierParameter2::Exact(_) => Ok(Classification::Decided(None)),
            BezierParameter2::Algebraic(parameter) => {
                BezierAlgebraicEndpointImage2::from_source_curve(
                    &offset_subcurve,
                    parameter,
                    policy,
                )
                .map(|image| image.map(Some))
            }
        }
    };
    let start_image = match endpoint_image(start)? {
        Classification::Decided(image) => image,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_image = match endpoint_image(end)? {
        Classification::Decided(image) => image,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let offset_rational = RationalBezier2::try_from_subcurve(&offset_subcurve)?;
    let offset_start = match crate::rational_bezier_general::exact_contact_point_evidence(
        &offset_rational,
        traversal_start,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let offset_end = match crate::rational_bezier_general::exact_contact_point_evidence(
        &offset_rational,
        traversal_end,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let start_tangent = match exact_circular_algebraic_endpoint_tangent(
        &offset_rational,
        traversal_start,
        &offset_start,
        &offset_circle,
        source_arc.is_clockwise(),
        reversed,
        policy,
    ) {
        Classification::Decided(tangent) => tangent,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let end_tangent = match exact_circular_algebraic_endpoint_tangent(
        &offset_rational,
        traversal_end,
        &offset_end,
        &offset_circle,
        source_arc.is_clockwise(),
        reversed,
        policy,
    ) {
        Classification::Decided(tangent) => tangent,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(vec![ExactOffsetSpan2 {
        fragments: vec![BezierSplitFragment2::RetainedBezier {
            reversed,
            start: start.clone(),
            end: end.clone(),
            source_curve: offset_subcurve,
            start_image,
            end_image,
        }],
        source_end,
        offset_start,
        offset_end,
        start_tangent,
        end_tangent,
    }]))
}

fn exact_offset_spans_from_source_singular_parallel(
    curve: &BezierSubcurve2,
    parallel: &BezierParallel2,
    analysis: &crate::BezierParallelSingularityAnalysis2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactOffsetSpan2>>> {
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    let mut source_boundaries = vec![(zero.clone(), false)];
    for singularity in analysis.source_singularities() {
        let after_zero = match singularity.cmp_by_refinement(&zero, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let before_one = match singularity.cmp_by_refinement(&one, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match (after_zero, before_one) {
            (std::cmp::Ordering::Equal, _) => source_boundaries[0].1 = true,
            (_, std::cmp::Ordering::Equal) => {}
            (std::cmp::Ordering::Greater, std::cmp::Ordering::Less) => {
                if let Some((previous, _)) = source_boundaries.last() {
                    match singularity.cmp_by_refinement(previous, policy)? {
                        Classification::Decided(std::cmp::Ordering::Greater) => {}
                        Classification::Decided(std::cmp::Ordering::Equal) => continue,
                        Classification::Decided(std::cmp::Ordering::Less) => {
                            return Err(CurveError::Topology(
                                "source singularities were not ordered".into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                source_boundaries.push((singularity.clone(), true));
            }
            _ => {
                return Err(CurveError::Topology(
                    "source singularity escaped the unit parameter domain".into(),
                ));
            }
        }
    }
    let end_is_singular = analysis.source_singularities().iter().any(|singularity| {
        matches!(
            singularity.cmp_by_refinement(&one, policy),
            Ok(Classification::Decided(std::cmp::Ordering::Equal))
        )
    });
    source_boundaries.push((one, end_is_singular));

    let source_rational = RationalBezier2::try_from_subcurve(curve)?;
    let mut spans = Vec::with_capacity(source_boundaries.len().saturating_sub(1));
    for branch in source_boundaries.windows(2) {
        let source_range =
            BezierParameterRange2::new_validated(branch[0].0.clone(), branch[1].0.clone());
        let mut boundaries = vec![(branch[0].0.clone(), branch[0].1)];
        for cusp in analysis.parallel_cusps() {
            let after_start = match cusp.cmp_by_refinement(source_range.start(), policy)? {
                Classification::Decided(order) => order.is_gt(),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let before_end = match cusp.cmp_by_refinement(source_range.end(), policy)? {
                Classification::Decided(order) => order.is_lt(),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if after_start && before_end {
                boundaries.push((cusp.clone(), false));
            }
        }
        boundaries.push((branch[1].0.clone(), branch[1].1));
        let ranges = boundaries
            .windows(2)
            .map(|pair| BezierParameterRange2::new_validated(pair[0].0.clone(), pair[1].0.clone()))
            .collect::<Vec<_>>();
        let start_scale = match parallel.regular_fragment_derivative_scale_sign(
            ranges
                .first()
                .expect("a source branch has one regular offset range"),
            policy,
        )? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_scale = match parallel.regular_fragment_derivative_scale_sign(
            ranges
                .last()
                .expect("a source branch has one regular offset range"),
            policy,
        )? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let endpoint = |parameter: &BezierParameter2,
                        singular: bool,
                        scale: RealSign|
         -> CurveResult<Classification<(CurvePoint2, CurveTangent2)>> {
            if singular {
                return parallel
                    .regular_source_point_and_tangent_support(
                        parallel,
                        &parameter.clone().into(),
                        &CurveParameterRange2::from_bezier_range(source_range.clone()),
                        scale,
                        policy,
                    )
                    .map(|frame| {
                        frame
                            .map(|(point, tangent)| (point, CurveTangent2::AlgebraicChord(tangent)))
                    });
            }
            let point = match exact_parallel_point_evidence(parallel, parameter, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let Some(parameter) = parameter.scalar() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let tangent = match parallel.derivative_at(parameter, policy)? {
                Classification::Decided(derivative) => CurveTangent2::RepresentedDirection((
                    derivative.dx().clone(),
                    derivative.dy().clone(),
                )),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(Classification::Decided((point, tangent)))
        };
        let (offset_start, start_tangent) =
            match endpoint(source_range.start(), branch[0].1, start_scale)? {
                Classification::Decided(endpoint) => endpoint,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let (offset_end, end_tangent) = match endpoint(source_range.end(), branch[1].1, end_scale)?
        {
            Classification::Decided(endpoint) => endpoint,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let mut endpoint_points = Vec::with_capacity(boundaries.len());
        for (index, (parameter, singular)) in boundaries.iter().enumerate() {
            if index == 0 {
                endpoint_points.push(offset_start.clone());
            } else if index + 1 == boundaries.len() {
                endpoint_points.push(offset_end.clone());
            } else if *singular {
                return Err(CurveError::Topology(
                    "a source singularity remained inside one regular branch".into(),
                ));
            } else {
                match exact_parallel_point_evidence(parallel, parameter, policy)? {
                    Classification::Decided(point) => endpoint_points.push(point),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        let fragments = boundaries
            .windows(2)
            .zip(endpoint_points.windows(2))
            .map(|(parameters, points)| {
                if parameters[0].1 || parameters[1].1 {
                    BezierSplitFragment2::SelectedFiber(
                        crate::bezier_split::BezierSelectedFiberFragment2::new(
                            BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
                            CurveParameterRange2::new_validated(
                                CurveParameter2::from(parameters[0].0.clone()),
                                CurveParameter2::from(parameters[1].0.clone()),
                            ),
                            points[0].clone(),
                            points[1].clone(),
                        ),
                    )
                } else {
                    BezierSplitFragment2::AnalyticParallel(
                        crate::BezierParallelFragment2::from_certified_range(
                            parallel.clone(),
                            BezierParameterRange2::new_validated(
                                parameters[0].0.clone(),
                                parameters[1].0.clone(),
                            ),
                            false,
                        ),
                    )
                }
            })
            .collect();
        let source_end = match crate::rational_bezier_general::exact_contact_point_evidence(
            &source_rational,
            source_range.end(),
            policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        spans.push(ExactOffsetSpan2 {
            fragments,
            source_end,
            offset_start,
            offset_end,
            start_tangent: Some(start_tangent),
            end_tangent: Some(end_tangent),
        });
    }
    // A stationary source parameter is not necessarily a geometric corner.
    // When both one-sided constructions prove the same offset endpoint and
    // codirected tangent, the common hodograph factor has preserved the normal
    // sheet. Keep the exact split fragments, but collapse their bookkeeping
    // into one span so stroke/offset composition does not Boolean two pieces of
    // one smooth band. Odd-multiplicity reversals have distinct nonzero-offset
    // endpoints (and opposite tangents at zero distance), so remain separate.
    let mut merged = Vec::<ExactOffsetSpan2>::with_capacity(spans.len());
    for span in spans {
        let joins_previous = merged.last().is_some_and(|previous| {
            policy.strict_predicate_pass(|| {
                previous.offset_end.same_point(&span.offset_start, policy)
                    == Classification::Decided(true)
                    && matches!(
                        previous.end_tangent.as_ref().zip(span.start_tangent.as_ref()),
                        Some((first, second))
                            if curve_tangent_cross_sign(first, second, policy)
                                == Classification::Decided(RealSign::Zero)
                                && curve_tangents_are_opposite(first, second, policy)
                                    == Classification::Decided(false)
                    )
            })
        });
        if joins_previous {
            let previous = merged
                .last_mut()
                .expect("a proven source-sheet join has a preceding span");
            previous.fragments.extend(span.fragments);
            previous.source_end = span.source_end;
            previous.offset_end = span.offset_end;
            previous.end_tangent = span.end_tangent;
        } else {
            merged.push(span);
        }
    }
    Ok(Classification::Decided(merged))
}

fn exact_offset_spans_from_materialized_curve(
    curve: &BezierSubcurve2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactOffsetSpan2>>> {
    if let Classification::Decided(segment) = materialized_native_subcurve_segment(curve, policy)? {
        let native_offset = match &segment {
            Segment2::Line(line) => {
                Classification::Decided(Segment2::Line(line.offset_left(distance.clone())?))
            }
            Segment2::Arc(arc) => {
                match exact_offset_span_from_native_arc(curve, arc, distance, policy)? {
                    Classification::Decided(span) => {
                        return Ok(Classification::Decided(vec![span]));
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
        };
        if let Classification::Decided(offset) = native_offset {
            return exact_offset_span_from_native_segment(curve, &offset, policy)
                .map(|span| span.map(|span| vec![span]));
        }
    }

    let source = match curve {
        BezierSubcurve2::Quadratic(curve) => BezierParallelSource2::Quadratic(curve.clone()),
        BezierSubcurve2::Cubic(curve) => BezierParallelSource2::Cubic(curve.clone()),
        BezierSubcurve2::RationalQuadratic(curve) => {
            BezierParallelSource2::Rational(curve.clone().into())
        }
        BezierSubcurve2::Rational(curve) => BezierParallelSource2::Rational(curve.clone()),
    };
    let parallel = BezierParallel2::from_source(source, distance.clone());
    let analysis = match parallel.singularity_analysis(&CurveParameterRange2::unit(), policy)? {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if !analysis.source_is_regular() {
        return exact_offset_spans_from_source_singular_parallel(
            curve, &parallel, &analysis, policy,
        );
    }

    let mut boundaries = Vec::with_capacity(analysis.parallel_cusps().len() + 2);
    boundaries.push(BezierParameter2::Exact(Real::zero()));
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    for cusp in analysis.parallel_cusps() {
        let after_zero = match cusp.cmp_by_refinement(&zero, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let before_one = match cusp.cmp_by_refinement(&one, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if after_zero == std::cmp::Ordering::Greater && before_one == std::cmp::Ordering::Less {
            let order = cusp.cmp_by_refinement(
                boundaries
                    .last()
                    .expect("parallel split inventory begins at zero"),
                policy,
            )?;
            match order {
                Classification::Decided(std::cmp::Ordering::Greater) => {
                    boundaries.push(cusp.clone());
                }
                Classification::Decided(std::cmp::Ordering::Equal) => {}
                Classification::Decided(std::cmp::Ordering::Less) => {
                    return Err(CurveError::Topology(
                        "parallel cusp isolators are not ordered".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }
    boundaries.push(one);

    let fragments = if boundaries.len() == 2 {
        match parallel.exact_pythagorean_hodograph_offset(policy)? {
            Classification::Decided(Some(offset)) => vec![materialized_offset_fragment(
                BezierSubcurve2::Rational(offset.curve().clone()),
            )],
            Classification::Decided(None) | Classification::Uncertain(_) => {
                exact_parallel_fragments(&parallel, &boundaries, false)
            }
        }
    } else {
        exact_parallel_fragments(&parallel, &boundaries, false)
    };
    let offset_start = match parallel.point_at(&Real::zero(), policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let offset_end = match parallel.point_at(&Real::one(), policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let start_tangent = match parallel.derivative_at(&Real::zero(), policy)? {
        Classification::Decided(derivative) => (derivative.dx().clone(), derivative.dy().clone()),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_tangent = match parallel.derivative_at(&Real::one(), policy)? {
        Classification::Decided(derivative) => (derivative.dx().clone(), derivative.dy().clone()),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(vec![ExactOffsetSpan2 {
        fragments,
        source_end: curve.end().clone().into(),
        offset_start: offset_start.into(),
        offset_end: offset_end.into(),
        start_tangent: Some(CurveTangent2::RepresentedDirection(start_tangent)),
        end_tangent: Some(CurveTangent2::RepresentedDirection(end_tangent)),
    }]))
}

fn exact_offset_span_from_native_arc(
    source: &BezierSubcurve2,
    arc: &CircularArc2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let radius_scale = arc.left_offset_radius_scale(distance)?;
    match real_sign(&radius_scale, policy) {
        Some(RealSign::Zero) => {
            let center = CurvePoint2::from(arc.center().clone());
            Ok(Classification::Decided(ExactOffsetSpan2 {
                fragments: Vec::new(),
                source_end: source.end().clone().into(),
                offset_start: center.clone(),
                offset_end: center,
                start_tangent: None,
                end_tangent: None,
            }))
        }
        Some(RealSign::Positive | RealSign::Negative) => {
            let scale_point = |point: &Point2| {
                let (delta_x, delta_y) = point.delta_from(arc.center());
                arc.center()
                    .translated(&delta_x * &radius_scale, &delta_y * &radius_scale)
            };
            let offset = CircularArc2::try_from_center_with_bulge(
                scale_point(arc.start()),
                scale_point(arc.end()),
                arc.center().clone(),
                arc.is_clockwise(),
                arc.bulge().cloned(),
            )?;
            exact_offset_span_from_native_segment(source, &Segment2::Arc(offset), policy)
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn exact_offset_span_from_algebraic_chord(
    chord: &crate::BezierAlgebraicChord2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    if let (Some(start), Some(end)) = (chord.start().coordinates(), chord.end().coordinates()) {
        // A normalized boundary can retain an ordinary represented line as a
        // chord. Keep its native offset, measurement, and output capabilities;
        // no selected coordinate or root is materialized by this branch.
        let line = LineSeg2::try_new(start.clone(), end.clone())?;
        let offset = Segment2::Line(line.offset_left(distance.clone())?);
        let source = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(line));
        return exact_offset_span_from_native_segment(&source, &offset, policy);
    }
    // Selected endpoints keep the common normal-displacement authority;
    // expanding them into unrelated coordinate expressions loses that proof.
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-span",
        "retained-oblique-algebraic-chord",
    );
    let offset_chord = chord.parallel_left_retained(distance.clone(), policy)?;
    let offset_start = offset_chord.start().clone();
    let offset_end = offset_chord.end().clone();
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments: vec![retained_chord_fragment(offset_chord)],
        source_end: chord.end().clone(),
        offset_start,
        offset_end,
        start_tangent: Some(CurveTangent2::AlgebraicChord(chord.clone())),
        end_tangent: Some(CurveTangent2::AlgebraicChord(chord.clone())),
    }))
}

fn exact_algebraic_cusp_semicircle_endpoint(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    at_start: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    match fragment.endpoint_point_evidence(at_start, policy)? {
        Classification::Decided(Some(point)) => Ok(Classification::Decided(point)),
        Classification::Decided(None) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn exact_offset_algebraic_cusp_semicircle_endpoint(
    source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    offset: &crate::BezierAlgebraicCuspSemicircleFragment2,
    source_endpoint: &CurvePoint2,
    at_start: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    match source.translated_cardinal_offset_endpoint(offset, at_start, source_endpoint, policy)? {
        Classification::Decided(Some(point)) => Ok(Classification::Decided(point)),
        Classification::Decided(None) => {
            match source.concentric_offset_endpoint_point_evidence(offset, at_start, policy)? {
                Classification::Decided(Some(point)) => Ok(Classification::Decided(point)),
                Classification::Decided(None) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-selected-circle-endpoint",
                        "general-evaluation",
                    );
                    exact_algebraic_cusp_semicircle_endpoint(offset, at_start, policy)
                }
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-selected-circle-endpoint",
                        "concentric-uncertain",
                    );
                    Ok(Classification::Uncertain(reason))
                }
            }
        }
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-endpoint",
                "cardinal-translation-uncertain",
            );
            Ok(Classification::Uncertain(reason))
        }
    }
}

fn selected_circle_endpoint_tangent(
    source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    offset: &crate::BezierAlgebraicCuspSemicircleFragment2,
    at_start: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<CurveTangent2>>> {
    match offset.endpoint_chord_tangent_relation(at_start, policy)? {
        Classification::Decided(Some((chord, circle_cross_chord, circle_dot_chord))) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent",
                "selected-circle-chord-contact",
            );
            return Ok(Classification::Decided(Some(CurveTangent2::ChordContact {
                fragment: offset.clone(),
                at_start,
                chord,
                circle_cross_chord,
                circle_dot_chord,
            })));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let tangent = offset
        .represented_endpoint_tangent(at_start, policy)?
        .map(|tangent| {
            Some(tangent.map_or_else(
                || CurveTangent2::SelectedCircularEndpoint {
                    source_fragment: source.clone(),
                    fragment: offset.clone(),
                    at_start,
                },
                CurveTangent2::RepresentedDirection,
            ))
        });
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-tangent",
        match &tangent {
            Classification::Decided(Some(CurveTangent2::SelectedCircularEndpoint { .. })) => {
                "retained-selected-circle-endpoint"
            }
            Classification::Decided(Some(_)) => "represented-selected-circle-endpoint",
            Classification::Decided(None) => {
                unreachable!("mapped tangents always retain a carrier")
            }
            Classification::Uncertain(_) => "uncertain-selected-circle-endpoint",
        },
    );
    Ok(tangent)
}

fn exact_offset_span_from_algebraic_cusp_semicircle(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let source_start = match exact_algebraic_cusp_semicircle_endpoint(fragment, true, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-blocker",
                "source-start",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let source_end = match exact_algebraic_cusp_semicircle_endpoint(fragment, false, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-blocker",
                "source-end",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let offset_fragment = match fragment.offset_left(distance, policy)? {
        Classification::Decided(Some(fragment)) => fragment,
        Classification::Decided(None) => {
            // Every parameter maps to the selected center at the exact radius
            // collapse.  Retain that point at both span boundaries and emit
            // no degenerate curve; adjacent parallels can then meet there and
            // the authoritative regularizer sees the lower-complexity loop.
            let center = match fragment.semicircle().center_point_evidence(policy)? {
                Classification::Decided(center) => center,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Ok(Classification::Decided(ExactOffsetSpan2 {
                fragments: Vec::new(),
                source_end,
                offset_start: center.clone(),
                offset_end: center,
                start_tangent: None,
                end_tangent: None,
            }));
        }
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-blocker",
                "offset-carrier",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let offset_start = match exact_offset_algebraic_cusp_semicircle_endpoint(
        fragment,
        &offset_fragment,
        &source_start,
        true,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-blocker",
                "offset-start",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let offset_end = match exact_offset_algebraic_cusp_semicircle_endpoint(
        fragment,
        &offset_fragment,
        &source_end,
        false,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-blocker",
                "offset-end",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let start_tangent =
        match selected_circle_endpoint_tangent(fragment, &offset_fragment, true, policy)? {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-selected-circle-blocker",
                    "start-tangent",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
    let end_tangent =
        match selected_circle_endpoint_tangent(fragment, &offset_fragment, false, policy)? {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-selected-circle-blocker",
                    "end-tangent",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments: vec![BezierSplitFragment2::AlgebraicCuspSemicircle(
            offset_fragment,
        )],
        source_end,
        offset_start,
        offset_end,
        start_tangent,
        end_tangent,
    }))
}

fn analytic_parallel_traversal_start(
    fragment: &crate::BezierParallelFragment2,
) -> &BezierParameter2 {
    if fragment.is_reversed() {
        fragment.range().end()
    } else {
        fragment.range().start()
    }
}

fn analytic_parallel_traversal_end(fragment: &crate::BezierParallelFragment2) -> &BezierParameter2 {
    if fragment.is_reversed() {
        fragment.range().start()
    } else {
        fragment.range().end()
    }
}

#[derive(Clone, Copy)]
enum RetainedParallelOffsetFragmentRef2<'a> {
    Analytic(&'a crate::BezierParallelFragment2),
    Selected(&'a crate::bezier_split::BezierSelectedFiberFragment2),
}

impl<'a> RetainedParallelOffsetFragmentRef2<'a> {
    fn from_fragment(fragment: &'a BezierSplitFragment2) -> Option<Self> {
        match fragment {
            BezierSplitFragment2::AnalyticParallel(fragment) => Some(Self::Analytic(fragment)),
            BezierSplitFragment2::SelectedFiber(fragment) => Some(Self::Selected(fragment)),
            BezierSplitFragment2::Materialized { .. }
            | BezierSplitFragment2::RetainedBezier { .. }
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
        }
    }

    fn parallel(self) -> BezierParallel2 {
        match self {
            Self::Analytic(fragment) => fragment.parallel().clone(),
            Self::Selected(fragment) => match fragment.source() {
                BezierSelectedFiberSource2::Rational(curve) => BezierParallel2::from_source(
                    BezierParallelSource2::Rational(curve.clone()),
                    Real::zero(),
                ),
                BezierSelectedFiberSource2::AnalyticParallel(parallel) => parallel.clone(),
            },
        }
    }

    fn range(self) -> CurveParameterRange2 {
        match self {
            Self::Analytic(fragment) => {
                CurveParameterRange2::from_bezier_range(fragment.range().clone())
            }
            Self::Selected(fragment) => fragment.range().clone(),
        }
    }

    fn is_reversed(self) -> bool {
        match self {
            Self::Analytic(fragment) => fragment.is_reversed(),
            Self::Selected(fragment) => fragment.is_reversed(),
        }
    }

    fn same_carrier(self, other: Self) -> bool {
        match (self, other) {
            (Self::Analytic(first), Self::Analytic(second)) => {
                first.parallel() == second.parallel()
            }
            (Self::Selected(first), Self::Selected(second)) => first.source() == second.source(),
            (Self::Analytic(first), Self::Selected(second))
            | (Self::Selected(second), Self::Analytic(first)) => match second.source() {
                BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
                    first.parallel() == parallel
                }
                BezierSelectedFiberSource2::Rational(curve) => {
                    first.parallel().distance() == &Real::zero()
                        && matches!(
                            first.parallel().source(),
                            BezierParallelSource2::Rational(source) if source == curve
                        )
                }
            },
        }
    }
}

fn retained_parallel_represented_parameter(parameter: &CurveParameter2) -> Option<&Real> {
    parameter.scalar().or_else(|| {
        parameter
            .as_selected_fiber()
            .and_then(|parameter| parameter.represented_value())
    })
}

fn retained_parallel_traversal_start(
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
) -> CurveParameter2 {
    let range = fragment.range();
    if fragment.is_reversed() {
        range.end().clone()
    } else {
        range.start().clone()
    }
}

fn retained_parallel_traversal_end(
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
) -> CurveParameter2 {
    let range = fragment.range();
    if fragment.is_reversed() {
        range.start().clone()
    } else {
        range.end().clone()
    }
}

fn exact_retained_parallel_fragment(
    parallel: BezierParallel2,
    range: &CurveParameterRange2,
    reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Option<crate::BezierParallelFragment2>> {
    let promote = |parameter: &CurveParameter2| -> CurveResult<Option<BezierParameter2>> {
        if let Some(parameter) = parameter.as_bezier_parameter() {
            return Ok(Some(parameter.clone()));
        }
        let Some(parameter) = parameter.as_selected_fiber() else {
            return Ok(None);
        };
        Ok(match parameter.promoted_bezier_parameter(policy)? {
            Classification::Decided(parameter) => Some(parameter),
            Classification::Uncertain(_) => None,
        })
    };
    let Some(start) = promote(range.start())? else {
        return Ok(None);
    };
    let Some(end) = promote(range.end())? else {
        return Ok(None);
    };
    Ok(Some(crate::BezierParallelFragment2::from_certified_range(
        parallel,
        BezierParameterRange2::new_validated(start, end),
        reversed,
    )))
}

fn exact_parallel_region_point_evidence(
    parallel: &BezierParallel2,
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    if let Some(parameter) = parameter.as_bezier_parameter() {
        return exact_parallel_point_evidence(parallel, parameter, policy);
    }
    let Some(point) =
        crate::BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
            parallel.clone(),
            parameter,
            Real::zero(),
            policy,
        )
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    Ok(Classification::Decided(CurvePoint2::from(point)))
}

fn exact_parallel_region_endpoint_tangent(
    parallel: &BezierParallel2,
    source_parallel: &BezierParallel2,
    source_range: &CurveParameterRange2,
    parameter: &CurveParameter2,
    scale: RealSign,
    reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveTangent2>> {
    debug_assert_ne!(scale, RealSign::Zero);
    let source_direction = if (scale == RealSign::Positive) != reversed {
        RealSign::Positive
    } else {
        RealSign::Negative
    };
    if let Some(parameter) = parameter.as_bezier_parameter() {
        return exact_parallel_endpoint_tangent(
            parallel,
            source_parallel,
            source_range,
            parameter,
            scale,
            reversed,
        );
    }
    if let Some(selected_source_parameter) = parameter.as_selected_fiber()
        && let Classification::Decided(parameter) =
            selected_source_parameter.promoted_bezier_parameter(policy)?
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-tangent",
            "selected-fiber-retained-parallel",
        );
        return Ok(Classification::Decided(CurveTangent2::RetainedParallel {
            parallel: parallel.clone(),
            source_parallel: source_parallel.clone(),
            source_range: source_range.clone(),
            parameter,
            selected_source_parameter: Some(selected_source_parameter.clone()),
            source_direction,
        }));
    }
    // The tangent direction is shared by every signed parallel of one source,
    // but a round join is centered on the unoffset boundary point. Retaining
    // the composed parallel here would move that center by the offset a
    // second time when the chord-normal fallback constructs its circle.
    Ok(
        crate::BezierAlgebraicChord2::from_certified_retained_parallel_oriented_unit_tangent(
            source_parallel.clone(),
            parameter,
            source_direction,
            policy,
        )?
        .map(CurveTangent2::AlgebraicChord),
    )
}

fn promoted_endpoint_image_corner_fragment(
    fragment: &BezierSplitFragment2,
    operation: CurveOperation2,
) -> ExactCurveResult<crate::BezierParallelFragment2> {
    let BezierSplitFragment2::RetainedBezier {
        reversed,
        start,
        end,
        source_curve: source,
        ..
    } = fragment
    else {
        return Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        ));
    };
    // Endpoint-image fragments already carry a validated source interval and
    // first-order endpoint evidence. Re-enter corner editing as that source's
    // exact zero-distance parallel, so every later trim or extension uses the
    // same retained carrier instead of growing another endpoint-field engine.
    let parallel = retained_subcurve_parallel(source, Real::zero())
        .map_err(|cause| curve_region_edit_error(operation, cause))?;
    Ok(crate::BezierParallelFragment2::from_certified_range(
        parallel,
        BezierParameterRange2::new_validated(start.clone(), end.clone()),
        *reversed,
    ))
}

/// Promotes an algebraic-endpoint fragment whose complete image is a
/// certified line segment to the shared infinite affine-line carrier.
///
/// The endpoint images remain the finite chord boundaries while the retained
/// source fit supplies the complete support. This is the exact extension
/// authority for line-image endpoint fragments; nonlinear sources continue
/// through their analytic zero-distance parallel instead.
fn promoted_endpoint_image_corner_chord(
    fragment: &BezierSplitFragment2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<crate::BezierAlgebraicChord2>> {
    let BezierSplitFragment2::RetainedBezier {
        source_curve: source,
        ..
    } = fragment
    else {
        return Ok(None);
    };
    let line = match subcurve_fit_exact_line_image(source, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => fit.line().clone(),
        Classification::Decided(BezierLineImageFitRelation::NotLine)
        | Classification::Uncertain(_) => return Ok(None),
    };
    let endpoint = |start_endpoint| -> ExactCurveResult<_> {
        match curve_fragment_endpoint_point(fragment, start_endpoint, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(Some(point)) => Ok(point),
            Classification::Decided(None) => Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            )),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                reason,
            )),
        }
    };
    let support = retained_algebraic_line_support(&line, operation, policy)?;
    match support
        .chord_between_certified_support_points(endpoint(true)?, endpoint(false)?, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(Some(chord)) => Ok(Some(chord)),
        Classification::Decided(None) => Err(curve_region_edit_error(
            operation,
            CurveError::Topology(
                "an algebraic-endpoint line-image corner collapsed to one point".into(),
            ),
        )),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            reason,
        )),
    }
}

/// One admitted retained boundary fragment and the exact carrier evidence
/// shared by region chamfer and fillet solving.
///
/// Admission deliberately precedes algebraic promotion so design-value
/// validation keeps its public ordering (most notably a zero-radius fillet).
/// `prepare` then promotes at most once and owns that evidence through solving
/// and retained publication.
pub(crate) struct CornerCarrierPreparation2<'a> {
    top_level: Option<std::borrow::Cow<'a, Curve2>>,
    fragment: Option<&'a BezierSplitFragment2>,
    evidence: CornerCarrierEvidence2,
    source_endpoint_is_end: bool,
    source_chart: Option<(&'a Real, &'a Real)>,
}

enum CornerCarrierEvidence2 {
    Source,
    Chord(crate::BezierAlgebraicChord2),
    Parallel(crate::BezierParallelFragment2),
    Circular(std::sync::Arc<crate::curve::RetainedRationalCornerArc2>),
}

impl<'a> CornerCarrierPreparation2<'a> {
    fn admit(fragment: &'a BezierSplitFragment2) -> Self {
        let top_level = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                Some(std::borrow::Cow::Owned(Curve2::from(curve.clone())))
            }
            BezierSplitFragment2::RetainedBezier { .. }
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::SelectedFiber(_) => None,
        };
        Self {
            top_level,
            fragment: Some(fragment),
            evidence: CornerCarrierEvidence2::Source,
            source_endpoint_is_end: false,
            source_chart: None,
        }
    }

    pub(crate) fn from_curve(curve: &'a Curve2, previous: bool) -> Self {
        if let Some(fragment) = curve.retained_fragment() {
            return Self::admit(fragment);
        }
        Self {
            top_level: Some(std::borrow::Cow::Borrowed(curve)),
            fragment: None,
            evidence: CornerCarrierEvidence2::Source,
            source_endpoint_is_end: previous,
            source_chart: None,
        }
    }

    fn family(&self) -> CurveFamily2 {
        self.top_level
            .as_ref()
            .map_or(CurveFamily2::RationalBezier, |curve| curve.family())
    }

    pub(crate) fn prepare(
        &mut self,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<()> {
        let source = match &self.top_level {
            Some(std::borrow::Cow::Borrowed(curve)) => Some(*curve),
            _ => None,
        };
        if let Some(source) = source
            && let Some(spans) = source.restricted_source_spans(policy, operation)?
        {
            let span = if self.source_endpoint_is_end {
                spans.last()
            } else {
                spans.first()
            }
            .expect("a source restriction contains a span");
            *self = Self::admit(&span.fragment);
            self.source_chart = Some((&span.source_scale, &span.source_offset));
        }
        if let Some(curve) = self.top_level.as_ref() {
            if let Some(crate::curve::ExactCornerCarrier2::RetainedRationalArc(arc)) =
                exact_corner_carrier(curve, self.source_endpoint_is_end, operation, policy)?
            {
                self.evidence = CornerCarrierEvidence2::Circular(arc);
            }
            return Ok(());
        }
        let fragment = self
            .fragment
            .expect("a nonnative corner retains its fragment");
        if let Some(arc) =
            crate::curve::RetainedRationalCornerArc2::from_fragment(fragment, operation, policy)?
        {
            self.evidence = CornerCarrierEvidence2::Circular(arc);
            return Ok(());
        }
        match fragment {
            BezierSplitFragment2::RetainedBezier { .. } => {
                self.evidence = if let Some(chord) =
                    promoted_endpoint_image_corner_chord(fragment, operation, policy)?
                {
                    CornerCarrierEvidence2::Chord(chord)
                } else {
                    CornerCarrierEvidence2::Parallel(promoted_endpoint_image_corner_fragment(
                        fragment, operation,
                    )?)
                };
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                if let Some(parallel) = exact_retained_parallel_fragment(
                    RetainedParallelOffsetFragmentRef2::Selected(fragment).parallel(),
                    fragment.range(),
                    fragment.is_reversed(),
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(operation, cause))?
                {
                    self.evidence = CornerCarrierEvidence2::Parallel(parallel);
                }
            }
            BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {}
            BezierSplitFragment2::Materialized { .. } => {
                return Err(ExactCurveError::blocked(
                    operation,
                    self.family(),
                    UncertaintyReason::Unsupported,
                ));
            }
        }
        Ok(())
    }

    pub(crate) fn exact_carrier(
        &self,
        previous: bool,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<crate::curve::ExactCornerCarrier2<'_>> {
        match &self.evidence {
            CornerCarrierEvidence2::Circular(arc) => {
                return Ok(crate::curve::ExactCornerCarrier2::RetainedRationalArc(
                    std::sync::Arc::clone(arc),
                ));
            }
            CornerCarrierEvidence2::Chord(chord) => {
                return Ok(crate::curve::ExactCornerCarrier2::AlgebraicChord(chord));
            }
            CornerCarrierEvidence2::Parallel(parallel) => {
                return Ok(crate::curve::ExactCornerCarrier2::AnalyticParallel(
                    parallel,
                ));
            }
            CornerCarrierEvidence2::Source => {}
        }
        if let Some(curve) = self.top_level.as_ref() {
            return exact_corner_carrier(curve, previous, operation, policy)?.ok_or_else(|| {
                ExactCurveError::blocked(operation, self.family(), UncertaintyReason::Unsupported)
            });
        }
        let fragment = self
            .fragment
            .expect("a nonnative corner retains its fragment");
        match fragment {
            BezierSplitFragment2::AlgebraicChord(chord) => {
                Ok(crate::curve::ExactCornerCarrier2::AlgebraicChord(chord))
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => Ok(
                crate::curve::ExactCornerCarrier2::AnalyticParallel(fragment),
            ),
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                Ok(crate::curve::ExactCornerCarrier2::AlgebraicCusp(fragment))
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                Ok(crate::curve::ExactCornerCarrier2::SelectedFiber(fragment))
            }
            BezierSplitFragment2::RetainedBezier { .. }
            | BezierSplitFragment2::Materialized { .. } => Err(ExactCurveError::blocked(
                operation,
                self.family(),
                UncertaintyReason::Unsupported,
            )),
        }
    }

    pub(crate) fn promoted_parallel(&self) -> Option<&crate::BezierParallelFragment2> {
        match &self.evidence {
            CornerCarrierEvidence2::Parallel(parallel) => Some(parallel),
            _ => None,
        }
    }

    pub(crate) fn source_chart(&self) -> Option<(&Real, &Real)> {
        self.source_chart
    }
}

fn retained_parallel_range_scale_sign(
    parallel: &BezierParallel2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let parameter = match range
        .start()
        .strict_scalar_between_ordered(range.end(), policy)?
    {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    parallel.parallel_derivative_scale_sign(&parameter.into(), policy)
}

fn exact_offset_spans_from_retained_parallel_fragment(
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactOffsetSpan2>>> {
    let parallel = fragment.parallel();
    let range = fragment.range();
    let analysis = match parallel.singularity_analysis(&range, policy)? {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut ranges = match analysis.regular_subranges(policy)? {
        Classification::Decided(ranges) => ranges,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if fragment.is_reversed() {
        ranges.reverse();
    }
    let mut spans = Vec::with_capacity(ranges.len());
    for (index, range) in ranges.iter().enumerate() {
        let source_end = match fragment {
            RetainedParallelOffsetFragmentRef2::Selected(fragment) if index + 1 == ranges.len() => {
                Some(fragment.end_point())
            }
            _ => None,
        };
        match exact_offset_span_from_regular_parallel_range(
            &parallel,
            range,
            fragment.is_reversed(),
            source_end,
            distance,
            policy,
        )? {
            Classification::Decided(span) => spans.push(span),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(spans))
}

fn exact_offset_span_from_regular_parallel_range(
    parallel: &BezierParallel2,
    range: &CurveParameterRange2,
    reversed: bool,
    source_end: Option<&CurvePoint2>,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let source_scale = match retained_parallel_range_scale_sign(parallel, range, policy)? {
        Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let traversal_agrees_with_source = (source_scale == RealSign::Positive) != reversed;
    let composed_distance = if traversal_agrees_with_source {
        parallel.distance() + distance
    } else {
        parallel.distance() - distance
    };
    let composed = parallel.with_distance(composed_distance);
    let composed_distance_sign = match real_sign(composed.distance(), policy) {
        Some(sign) => sign,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };

    // A retained endpoint may be the finite one-sided limit of a stationary
    // source. Its canonical source root supplies the primitive tangent frame;
    // every other cut stays in its original parameter authority.
    let mut source_endpoints = [None, None];
    let analysis = match composed.singularity_analysis(range, policy) {
        Ok(Classification::Decided(analysis)) => Some(analysis),
        // Zero displacement already defines the source without a unit normal.
        // Optional endpoint-frame recovery must not narrow that existing domain.
        _ if composed_distance_sign == RealSign::Zero => None,
        Ok(Classification::Uncertain(reason)) => return Ok(Classification::Uncertain(reason)),
        Err(error) => return Err(error),
    };
    let ranges = if let Some(analysis) = analysis {
        for singularity in analysis.source_singularities() {
            let parameter = CurveParameter2::from(singularity.clone());
            let after_start = match parameter.cmp_by_refinement(range.start(), policy) {
                Ok(Classification::Decided(order)) => order,
                _ if composed_distance_sign == RealSign::Zero => continue,
                Ok(Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
                Err(error) => return Err(error),
            };
            let before_end = match parameter.cmp_by_refinement(range.end(), policy) {
                Ok(Classification::Decided(order)) => order,
                _ if composed_distance_sign == RealSign::Zero => continue,
                Ok(Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
                Err(error) => return Err(error),
            };
            if after_start.is_eq() {
                source_endpoints[0] = Some(singularity.clone());
            } else if before_end.is_eq() {
                source_endpoints[1] = Some(singularity.clone());
            } else if after_start.is_gt()
                && before_end.is_lt()
                && composed_distance_sign != RealSign::Zero
            {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        }
        match analysis.regular_subranges(policy)? {
            Classification::Decided(ranges) => ranges,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    } else {
        vec![range.clone()]
    };
    let boundaries = ranges
        .iter()
        .map(|range| range.start().clone())
        .chain(std::iter::once(range.end().clone()))
        .collect::<Vec<_>>();
    let (start_index, end_index) = if reversed { (1, 0) } else { (0, 1) };
    let endpoint = |index: usize,
                    scale: RealSign|
     -> CurveResult<Classification<(CurvePoint2, CurveTangent2)>> {
        if let Some(parameter) = &source_endpoints[index] {
            let direction = if reversed {
                match scale {
                    RealSign::Positive => RealSign::Negative,
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => unreachable!(),
                }
            } else {
                scale
            };
            // The point belongs to the new offset, but a join's tangent support
            // remains anchored at the original region corner.
            let limit = composed.regular_source_point_and_tangent_support(
                parallel,
                &parameter.clone().into(),
                range,
                direction,
                policy,
            );
            if composed_distance_sign != RealSign::Zero
                || matches!(&limit, Ok(Classification::Decided(_)))
            {
                return limit.map(|result| {
                    result.map(|(point, tangent)| (point, CurveTangent2::AlgebraicChord(tangent)))
                });
            }
        }
        let parameter = if index == 0 {
            range.start()
        } else {
            range.end()
        };
        let point = match exact_parallel_region_point_evidence(&composed, parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(exact_parallel_region_endpoint_tangent(
            &composed, parallel, range, parameter, scale, reversed, policy,
        )?
        .map(|tangent| (point, tangent)))
    };
    // Join tangents belong to the unoffset corner. On this regular source
    // range the boundary keeps one derivative-scale sign; the composed
    // parallel's sign can differ beside a boundary cusp, where the offset
    // crosses the curvature radius, and must not orient the corner.
    let (offset_start, start_tangent) = match endpoint(start_index, source_scale)? {
        Classification::Decided(endpoint) => endpoint,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let (offset_end, end_tangent) = match endpoint(end_index, source_scale)? {
        Classification::Decided(endpoint) => endpoint,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let source_end = if let Some(point) = source_end {
        point.clone()
    } else if source_endpoints[end_index].is_some()
        && let CurveTangent2::AlgebraicChord(tangent) = &end_tangent
    {
        tangent.start().clone()
    } else {
        let parameter = if end_index == 0 {
            range.start()
        } else {
            range.end()
        };
        match exact_parallel_region_point_evidence(parallel, parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    };

    // Share each cut point between its incident fragments. Only cuts that need
    // selected parameters or one-sided source limits retain endpoint payloads.
    let mut points = Vec::new();
    let mut fragments = Vec::with_capacity(ranges.len());
    for (index, range) in ranges.iter().enumerate() {
        let has_source_limit = (index == 0 && source_endpoints[0].is_some())
            || (index + 1 == ranges.len() && source_endpoints[1].is_some());
        if !has_source_limit
            && let Some(fragment) =
                exact_retained_parallel_fragment(composed.clone(), range, reversed, policy)?
        {
            fragments.push(BezierSplitFragment2::AnalyticParallel(fragment));
        } else {
            if points.is_empty() {
                points.resize(boundaries.len(), None);
                points[0] = Some(if reversed {
                    offset_end.clone()
                } else {
                    offset_start.clone()
                });
                points[boundaries.len() - 1] = Some(if reversed {
                    offset_start.clone()
                } else {
                    offset_end.clone()
                });
            }
            for boundary in index..=index + 1 {
                if points[boundary].is_none() {
                    match exact_parallel_region_point_evidence(
                        &composed,
                        &boundaries[boundary],
                        policy,
                    )? {
                        Classification::Decided(point) => points[boundary] = Some(point),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            let selected = crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(composed.clone()),
                range.clone(),
                points[index]
                    .as_ref()
                    .expect("the selected start point was retained")
                    .clone(),
                points[index + 1]
                    .as_ref()
                    .expect("the selected end point was retained")
                    .clone(),
            );
            fragments.push(BezierSplitFragment2::SelectedFiber(if reversed {
                selected.reversed()
            } else {
                selected
            }));
        }
    }
    if reversed {
        fragments.reverse();
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-span",
        "retained-region-parameter",
    );
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments,
        source_end,
        offset_start,
        offset_end,
        start_tangent: Some(start_tangent),
        end_tangent: Some(end_tangent),
    }))
}

/// Coalesces one traversal-contiguous retained-parallel run whose only
/// non-represented boundaries are internal arrangement partitions.
///
/// Analytic-parallel and selected-fiber fragments share this authority. A
/// Boolean or self-contact split does not create a geometric corner, so a run
/// with one carrier and traversal is recovered between represented outer
/// endpoints only when its entire interior is certified regular. Cusps and
/// source singularities remain span boundaries, even without a sign change.
fn coalesced_retained_parallel_offset_run(
    fragments: &[BezierSplitFragment2],
    first_index: usize,
    maximum_run_length: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<(crate::BezierParallelFragment2, usize)>>> {
    if fragments.is_empty() || maximum_run_length == 0 {
        return Ok(Classification::Decided(None));
    }
    let Some(first) = fragments
        .get(first_index % fragments.len())
        .and_then(RetainedParallelOffsetFragmentRef2::from_fragment)
    else {
        return Ok(Classification::Decided(None));
    };
    let parallel = first.parallel();
    if retained_parallel_represented_parameter(&retained_parallel_traversal_start(first)).is_none()
    {
        return Ok(Classification::Decided(None));
    }
    let mut last = first;
    for step in 1..maximum_run_length.min(fragments.len()) {
        let next_index = (first_index + step) % fragments.len();
        let Some(next) = RetainedParallelOffsetFragmentRef2::from_fragment(&fragments[next_index])
        else {
            break;
        };
        if !first.same_carrier(next) || first.is_reversed() != next.is_reversed() {
            break;
        }
        match retained_parallel_traversal_end(last)
            .cmp_by_refinement(&retained_parallel_traversal_start(next), policy)?
        {
            Classification::Decided(std::cmp::Ordering::Equal) => {}
            Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                break;
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        last = next;
        let traversal_end = retained_parallel_traversal_end(last);
        if retained_parallel_represented_parameter(&traversal_end).is_some() {
            let first_range = first.range();
            let last_range = last.range();
            let (start, end) = if first.is_reversed() {
                (last_range.start(), first_range.end())
            } else {
                (first_range.start(), last_range.end())
            };
            let range = CurveParameterRange2::new_validated(start.clone(), end.clone());
            let analysis = match parallel.singularity_analysis(&range, policy)? {
                Classification::Decided(analysis) => analysis,
                Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
            };
            match analysis.regular_subranges(policy)? {
                Classification::Decided(ranges) if ranges.len() == 1 => {}
                Classification::Decided(_) | Classification::Uncertain(_) => {
                    return Ok(Classification::Decided(None));
                }
            }
            let start = retained_parallel_represented_parameter(start)
                .expect("the coalesced retained-parallel start is represented")
                .clone();
            let end = retained_parallel_represented_parameter(end)
                .expect("the coalesced retained-parallel end is represented")
                .clone();
            return Ok(Classification::Decided(Some((
                crate::BezierParallelFragment2::from_certified_range(
                    parallel,
                    BezierParameterRange2::new_validated(
                        BezierParameter2::Exact(start),
                        BezierParameter2::Exact(end),
                    ),
                    first.is_reversed(),
                ),
                step + 1,
            ))));
        }
    }
    Ok(Classification::Decided(None))
}

/// Coalesces one traversal-contiguous selected-circle run before offsetting.
///
/// Boolean arrangements may split a regular circular carrier at a mapped
/// contact that is not a geometric corner. Keeping that partition through the
/// unary arrangement repeats correlated parameter proofs and constructs two
/// identical concentric carriers. The fragment authority accepts only the
/// same carrier, traversal, and an exactly shared/equal cut; every other case
/// falls back to the ordinary per-fragment path.
fn coalesced_algebraic_circle_offset_run(
    fragments: &[BezierSplitFragment2],
    first_index: usize,
    maximum_run_length: usize,
    policy: &CurveContext,
) -> CurveResult<Option<(crate::BezierAlgebraicCuspSemicircleFragment2, usize)>> {
    if fragments.is_empty() || maximum_run_length == 0 {
        return Ok(None);
    }
    let Some(BezierSplitFragment2::AlgebraicCuspSemicircle(first)) =
        fragments.get(first_index % fragments.len())
    else {
        return Ok(None);
    };
    let mut coalesced = first.clone();
    let mut consumed = 1;
    while consumed < maximum_run_length.min(fragments.len()) {
        let Some(BezierSplitFragment2::AlgebraicCuspSemicircle(next)) =
            fragments.get((first_index + consumed) % fragments.len())
        else {
            break;
        };
        let Some(merged) = coalesced.coalesced_with_next(next, policy)? else {
            break;
        };
        coalesced = merged;
        consumed += 1;
    }
    Ok((consumed > 1).then_some((coalesced, consumed)))
}

fn exact_parallel_point_evidence(
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    if let Some(parameter) = parameter.scalar() {
        return Ok(parallel.point_at(parameter, policy)?.map(Into::into));
    }
    Ok(Classification::Decided(CurvePoint2::from(
        crate::BezierAnalyticParallelPoint2::new(parallel.clone(), parameter.clone(), policy),
    )))
}

fn exact_parallel_endpoint_tangent(
    parallel: &BezierParallel2,
    source_parallel: &BezierParallel2,
    source_range: &CurveParameterRange2,
    parameter: &BezierParameter2,
    scale: RealSign,
    reversed: bool,
) -> CurveResult<Classification<CurveTangent2>> {
    debug_assert_ne!(scale, RealSign::Zero);
    let source_direction = if (scale == RealSign::Positive) != reversed {
        RealSign::Positive
    } else {
        RealSign::Negative
    };
    Ok(Classification::Decided(CurveTangent2::RetainedParallel {
        parallel: parallel.clone(),
        source_parallel: source_parallel.clone(),
        source_range: source_range.clone(),
        parameter: parameter.clone(),
        selected_source_parameter: None,
        source_direction,
    }))
}

fn exact_parallel_fragments(
    parallel: &BezierParallel2,
    boundaries: &[BezierParameter2],
    reversed: bool,
) -> Vec<BezierSplitFragment2> {
    let mut fragments = boundaries
        .windows(2)
        .map(|pair| {
            BezierSplitFragment2::AnalyticParallel(
                crate::BezierParallelFragment2::from_certified_range(
                    parallel.clone(),
                    BezierParameterRange2::new_validated(pair[0].clone(), pair[1].clone()),
                    reversed,
                ),
            )
        })
        .collect::<Vec<_>>();
    if reversed {
        fragments.reverse();
    }
    fragments
}

fn materialized_offset_fragment(curve: BezierSubcurve2) -> BezierSplitFragment2 {
    BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve,
    }
}

fn exact_offset_span_from_native_segment(
    source: &BezierSubcurve2,
    offset: &Segment2,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let (fragments, start_tangent, end_tangent) = match offset {
        Segment2::Line(line) => {
            let tangent = line.delta();
            (
                vec![materialized_offset_fragment(BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(line.clone()),
                ))],
                tangent.clone(),
                tangent,
            )
        }
        Segment2::Arc(arc) => {
            let decomposition = match arc.rational_bezier_decomposition_with_policy(policy) {
                Ok(Classification::Decided(decomposition)) => decomposition,
                Ok(Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
                Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
                Err(ExactCurveError::Blocked(blocker)) => {
                    return Ok(Classification::Uncertain(blocker.reason()));
                }
            };
            let fragments = decomposition
                .spans()
                .iter()
                .map(|span| {
                    materialized_offset_fragment(BezierSubcurve2::RationalQuadratic(
                        span.curve().clone(),
                    ))
                })
                .collect();
            (
                fragments,
                native_segment_endpoint_tangent(offset, true),
                native_segment_endpoint_tangent(offset, false),
            )
        }
    };
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments,
        source_end: source.end().clone().into(),
        offset_start: offset.start().clone().into(),
        offset_end: offset.end().clone().into(),
        start_tangent: Some(CurveTangent2::RepresentedDirection(start_tangent)),
        end_tangent: Some(CurveTangent2::RepresentedDirection(end_tangent)),
    }))
}

fn exact_offset_span_from_source_run(
    source_fragments: &[BezierSplitFragment2],
    fragment_index: usize,
    remaining: usize,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<(Vec<ExactOffsetSpan2>, usize)>> {
    let fragment = &source_fragments[fragment_index];
    let mut consumed = 1;
    let offset = match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            exact_offset_spans_from_materialized_curve(curve, distance, policy)
        }
        BezierSplitFragment2::AnalyticParallel(_) | BezierSplitFragment2::SelectedFiber(_) => {
            match coalesced_retained_parallel_offset_run(
                source_fragments,
                fragment_index,
                remaining,
                policy,
            )? {
                Classification::Decided(Some((coalesced, run_length))) => {
                    consumed = run_length;
                    exact_offset_spans_from_retained_parallel_fragment(
                        RetainedParallelOffsetFragmentRef2::Analytic(&coalesced),
                        distance,
                        policy,
                    )
                }
                Classification::Decided(None) => {
                    exact_offset_spans_from_retained_parallel_fragment(
                        RetainedParallelOffsetFragmentRef2::from_fragment(fragment)
                            .expect("the retained-parallel match arm owns its view"),
                        distance,
                        policy,
                    )
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        BezierSplitFragment2::AlgebraicChord(chord) => {
            exact_offset_span_from_algebraic_chord(chord, distance, policy)
                .map(|span| span.map(|span| vec![span]))
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            match coalesced_algebraic_circle_offset_run(
                source_fragments,
                fragment_index,
                remaining,
                policy,
            )? {
                Some((coalesced, run_length)) => {
                    consumed = run_length;
                    exact_offset_span_from_algebraic_cusp_semicircle(&coalesced, distance, policy)
                        .map(|span| span.map(|span| vec![span]))
                }
                None => {
                    exact_offset_span_from_algebraic_cusp_semicircle(fragment, distance, policy)
                        .map(|span| span.map(|span| vec![span]))
                }
            }
        }
        BezierSplitFragment2::RetainedBezier {
            reversed,
            start,
            end,
            source_curve,
            ..
        } => exact_offset_spans_from_algebraic_endpoint_images(
            *reversed,
            start,
            end,
            source_curve,
            distance,
            policy,
        ),
    }?;
    Ok(offset.map(|span| (span, consumed)))
}

fn exact_offset_span_runs_from_boundary_loop(
    boundary_loop: &CurveRegionBoundaryLoop2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<(ExactOffsetSpan2, usize)>>> {
    let source_fragments = boundary_loop.fragments();
    let processing_start = match source_fragments
        .first()
        .and_then(RetainedParallelOffsetFragmentRef2::from_fragment)
    {
        Some(first)
            if retained_parallel_represented_parameter(&retained_parallel_traversal_start(
                first,
            ))
            .is_none() =>
        {
            source_fragments
                .iter()
                .position(|fragment| {
                    RetainedParallelOffsetFragmentRef2::from_fragment(fragment).is_some_and(
                        |candidate| {
                            retained_parallel_represented_parameter(
                                &retained_parallel_traversal_start(candidate),
                            )
                            .is_some()
                        },
                    )
                })
                .unwrap_or(0)
        }
        _ if matches!(
            source_fragments.first(),
            Some(BezierSplitFragment2::AlgebraicCuspSemicircle(first))
                if !first.traversal_start_parameter_is_exact()
        ) =>
        {
            source_fragments
                .iter()
                .position(|fragment| {
                    matches!(
                        fragment,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(candidate)
                            if candidate.traversal_start_parameter_is_exact()
                    )
                })
                .unwrap_or(0)
        }
        _ => 0,
    };
    let mut runs = Vec::with_capacity(boundary_loop.len());
    let mut processed = 0;
    while processed < source_fragments.len() {
        let fragment_index = (processing_start + processed) % source_fragments.len();
        let (spans, consumed) = match exact_offset_span_from_source_run(
            source_fragments,
            fragment_index,
            source_fragments.len() - processed,
            distance,
            policy,
        )? {
            Classification::Decided(span) => span,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                {
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-blocker",
                        "span",
                    );
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-span-blocker",
                        match &source_fragments[fragment_index] {
                            BezierSplitFragment2::Materialized { .. } => "materialized",
                            BezierSplitFragment2::AnalyticParallel(_) => "analytic-parallel",
                            BezierSplitFragment2::SelectedFiber(_) => "selected-fiber",
                            BezierSplitFragment2::AlgebraicChord(_) => "algebraic-chord",
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
                                "algebraic-cusp-semicircle"
                            }
                            BezierSplitFragment2::RetainedBezier { .. } => {
                                "algebraic-endpoint-images"
                            }
                        },
                    );
                }
                return Ok(Classification::Uncertain(reason));
            }
        };
        for (branch_index, span) in spans.into_iter().enumerate() {
            runs.push((span, if branch_index == 0 { consumed } else { 0 }));
        }
        processed += consumed;
    }
    Ok(Classification::Decided(runs))
}

fn native_segment_endpoint_tangent(segment: &Segment2, start: bool) -> (Real, Real) {
    match segment {
        Segment2::Line(line) => line.delta(),
        Segment2::Arc(arc) => {
            let point = if start { arc.start() } else { arc.end() };
            let (rx, ry) = point.delta_from(arc.center());
            if arc.is_clockwise() {
                (ry, -rx)
            } else {
                (-ry, rx)
            }
        }
    }
}

fn append_exact_offset_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    style: &OffsetCornerStyle2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let tangents = previous
        .end_tangent
        .as_ref()
        .zip(next.start_tangent.as_ref());
    let turn_sign = match tangents {
        Some((previous_tangent, next_tangent)) => {
            match curve_tangent_cross_sign(previous_tangent, next_tangent, policy) {
                Classification::Decided(sign) => Some(sign),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        None => None,
    };
    // A smooth carrier switch owns stronger pair evidence than two separately
    // materialized endpoint images. Consume that tangent/overlap proof first:
    // under APPROXIMATE_512, asking the generic point predicate first could
    // unnecessarily spend the terminal equality policy on an exactly smooth
    // join. Opposite or unresolved parallel tangents retain the point test.
    let mut tangents_opposite = None;
    if turn_sign == Some(RealSign::Zero)
        && let Some((previous_tangent, next_tangent)) = tangents
    {
        match curve_tangents_are_opposite(previous_tangent, next_tangent, policy) {
            Classification::Decided(false) => {
                // Boundary-loop construction already certified a shared
                // source vertex. Equal signed offsets along equal oriented
                // normals therefore share the exact offset vertex even when
                // the images inhabit independent selected fields.
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-join",
                    "smooth-source-overlap",
                );
                return Ok(Classification::Decided(()));
            }
            Classification::Decided(true) => {
                tangents_opposite = Some(true);
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-join",
                    "opposite-source-overlap",
                );
            }
            Classification::Uncertain(_) => {}
        }
    }
    if turn_sign.is_none_or(|sign| sign == RealSign::Zero) {
        match previous.offset_end.same_point(&next.offset_start, policy) {
            Classification::Decided(true) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-join",
                    "shared-endpoint",
                );
                return Ok(Classification::Decided(()));
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-join",
                    "endpoint-equality-uncertain",
                );
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let Some((previous_tangent, next_tangent)) = tangents else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-join",
            "missing-tangent",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let turn_sign = turn_sign.expect("retained tangent pair has one exact cross sign");
    let distance_sign = match real_sign(distance, policy) {
        Some(sign) => sign,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    let inward = exact_sign_product(turn_sign, distance_sign) == RealSign::Positive;
    #[cfg(feature = "dispatch-trace")]
    if inward {
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-inner-join-tangents",
            match (previous_tangent, next_tangent) {
                (
                    CurveTangent2::RepresentedDirection(_),
                    CurveTangent2::RepresentedDirection(_),
                ) => "vector-vector",
                (CurveTangent2::RepresentedDirection(_), CurveTangent2::ChordContact { .. }) => {
                    "vector-chord-contact"
                }
                (CurveTangent2::ChordContact { .. }, CurveTangent2::RepresentedDirection(_)) => {
                    "chord-contact-vector"
                }
                (
                    CurveTangent2::SelectedCircularEndpoint { .. },
                    CurveTangent2::SelectedCircularEndpoint { .. },
                ) => "selected-circle-selected-circle",
                (
                    CurveTangent2::SelectedCircularEndpoint { .. },
                    CurveTangent2::RepresentedDirection(_),
                ) => "selected-circle-vector",
                (
                    CurveTangent2::RepresentedDirection(_),
                    CurveTangent2::SelectedCircularEndpoint { .. },
                ) => "vector-selected-circle",
                (CurveTangent2::AlgebraicChord(_), CurveTangent2::AlgebraicChord(_)) => {
                    "algebraic-chord-algebraic-chord"
                }
                (CurveTangent2::AlgebraicChord(_), CurveTangent2::RepresentedDirection(_)) => {
                    "algebraic-chord-vector"
                }
                (CurveTangent2::RepresentedDirection(_), CurveTangent2::AlgebraicChord(_)) => {
                    "vector-algebraic-chord"
                }
                _ => "other-retained-pair",
            },
        );
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-join",
        match (style, inward) {
            (OffsetCornerStyle2::Round, false) => "round-outer",
            (OffsetCornerStyle2::Bevel, false) => "bevel-outer",
            (OffsetCornerStyle2::Miter { .. }, false) => "miter-outer",
            (OffsetCornerStyle2::Round, true) => "round-inner-miter",
            (OffsetCornerStyle2::Bevel, true) => "bevel-inner-miter",
            (OffsetCornerStyle2::Miter { .. }, true) => "miter-inner",
        },
    );
    match style {
        OffsetCornerStyle2::Round if !inward => {
            let opposite = match tangents_opposite {
                Some(opposite) => opposite,
                None => match curve_tangents_are_opposite(previous_tangent, next_tangent, policy) {
                    Classification::Decided(opposite) => opposite,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            append_exact_round_join(
                fragments,
                previous,
                next,
                distance,
                if opposite {
                    crate::arc_bezier::ArcSweepKind::Semicircle
                } else {
                    crate::arc_bezier::ArcSweepKind::Minor
                },
                policy,
            )
        }
        OffsetCornerStyle2::Bevel if !inward => append_exact_algebraic_line_join(
            fragments,
            &previous.offset_end,
            &next.offset_start,
            None,
            exact_offset_bevel_parameter_axis(
                previous_tangent,
                next_tangent,
                turn_sign,
                distance_sign,
                policy,
            ),
            turn_sign != RealSign::Zero,
            // For a nonzero turn, the difference of the two unit normals
            // cannot be parallel to either endpoint tangent. Thus this bevel
            // is strictly transverse to every selected-circle endpoint it
            // joins, independent of the represented offset distance.
            if turn_sign == RealSign::Zero {
                [false; 2]
            } else {
                [
                    exact_offset_tangent_is_selected_circle(previous_tangent),
                    exact_offset_tangent_is_selected_circle(next_tangent),
                ]
            },
            policy,
        ),
        OffsetCornerStyle2::Miter { limit } if !inward => append_exact_miter_join(
            fragments,
            previous,
            next,
            distance,
            Some(limit),
            turn_sign,
            distance_sign,
            policy,
        ),
        OffsetCornerStyle2::Round
        | OffsetCornerStyle2::Bevel
        | OffsetCornerStyle2::Miter { .. } => append_exact_miter_join(
            fragments,
            previous,
            next,
            distance,
            None,
            turn_sign,
            distance_sign,
            policy,
        ),
    }
}

fn exact_offset_join_band_semantics(
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<(bool, bool)>> {
    let endpoint_reason = match previous.offset_end.same_point(&next.offset_start, policy) {
        Classification::Decided(true) => {
            // A span that collapses to a point intentionally has no tangent.
            // Its adjacent offset spans already meet at the retained center,
            // so no corner band exists and no synthetic tangent is needed.
            return Ok(Classification::Decided((true, true)));
        }
        Classification::Decided(false) => None,
        Classification::Uncertain(reason) => Some(reason),
    };
    let Some((previous_tangent, next_tangent)) = previous
        .end_tangent
        .as_ref()
        .zip(next.start_tangent.as_ref())
    else {
        return Ok(Classification::Uncertain(
            endpoint_reason.unwrap_or(UncertaintyReason::Unsupported),
        ));
    };
    let turn = match curve_tangent_cross_sign(previous_tangent, next_tangent, policy) {
        Classification::Decided(turn) => turn,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let distance = match real_sign(distance, policy) {
        Some(distance) => distance,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    Ok(Classification::Decided((
        exact_sign_product(turn, distance) == RealSign::Positive,
        turn == RealSign::Positive,
    )))
}

fn exact_offset_spans_form_reversal(
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let Some((previous_tangent, next_tangent)) = previous
        .end_tangent
        .as_ref()
        .zip(next.start_tangent.as_ref())
    else {
        return Ok(Classification::Decided(false));
    };
    match curve_tangent_cross_sign(previous_tangent, next_tangent, policy) {
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {
            Ok(Classification::Decided(false))
        }
        Classification::Decided(RealSign::Zero) => Ok(curve_tangents_are_opposite(
            previous_tangent,
            next_tangent,
            policy,
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn exact_offset_band_connector(
    fragments: &mut Vec<BezierSplitFragment2>,
    from: &CurvePoint2,
    to: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    append_exact_algebraic_line_join(fragments, from, to, None, None, true, [false; 2], policy)
}

fn exact_offset_span_band_loop(
    opposite: &ExactOffsetSpan2,
    span: &ExactOffsetSpan2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveRegionBoundaryLoop2>> {
    let mut fragments = Vec::with_capacity(
        opposite
            .fragments
            .len()
            .saturating_add(span.fragments.len())
            .saturating_add(2),
    );
    fragments.extend(opposite.fragments.iter().cloned());
    match exact_offset_band_connector(
        &mut fragments,
        &opposite.offset_end,
        &span.offset_end,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    for fragment in span.fragments.iter().rev() {
        fragments.push(fragment.reversed()?);
    }
    match exact_offset_band_connector(
        &mut fragments,
        &span.offset_start,
        &opposite.offset_start,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(fragments, None, policy)
        .map(Classification::Decided)
}

fn exact_offset_corner_band_loop(
    source_vertex: &CurvePoint2,
    previous_offset_end: &CurvePoint2,
    join_fragments: Vec<BezierSplitFragment2>,
    next_offset_start: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveRegionBoundaryLoop2>> {
    if join_fragments.is_empty() {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let mut fragments = Vec::with_capacity(join_fragments.len().saturating_add(2));
    match exact_offset_band_connector(&mut fragments, source_vertex, previous_offset_end, policy)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    fragments.extend(join_fragments);
    match exact_offset_band_connector(&mut fragments, next_offset_start, source_vertex, policy)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(fragments, None, policy)
        .map(Classification::Decided)
}

fn exact_offset_span_runs_from_open_path(
    source_fragments: &[BezierSplitFragment2],
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactOffsetSpan2>>> {
    let mut spans = Vec::with_capacity(source_fragments.len());
    let mut processed = 0;
    while processed < source_fragments.len() {
        let (run, consumed) = match exact_offset_span_from_source_run(
            source_fragments,
            processed,
            source_fragments.len() - processed,
            distance,
            policy,
        )? {
            Classification::Decided(run) => run,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        spans.extend(run);
        processed += consumed;
    }
    Ok(Classification::Decided(spans))
}

fn exact_offset_corner_band(
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    style: &OffsetCornerStyle2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<(CurveRegionBoundaryLoop2, bool)>>> {
    let (inner_join, filled_side_is_left) =
        match exact_offset_join_band_semantics(previous, next, distance, policy)? {
            Classification::Decided(semantics) => semantics,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if inner_join {
        return Ok(Classification::Decided(None));
    }

    let mut join_fragments = Vec::new();
    match append_exact_offset_join(&mut join_fragments, previous, next, distance, style, policy)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    if join_fragments.is_empty() {
        return Ok(Classification::Decided(None));
    }
    exact_offset_corner_band_loop(
        &previous.source_end,
        &previous.offset_end,
        join_fragments,
        &next.offset_start,
        policy,
    )
    .map(|loop_| loop_.map(|loop_| Some((loop_, filled_side_is_left))))
}

fn regularized_exact_offset_band_arrangement(
    boundary_loops: Vec<CurveRegionBoundaryLoop2>,
    filled_sides_are_left: Vec<bool>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegion2> {
    if boundary_loops.is_empty() || boundary_loops.len() != filled_sides_are_left.len() {
        return Err(curve_region_edit_error(
            CurveOperation2::Offset,
            CurveError::Topology("exact offset produced inconsistent boundary bands".into()),
        ));
    }
    let band_count = boundary_loops.len();
    let mut band = CurveRegion2::new(boundary_loops)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
    {
        let data = band.data_mut_for_construction();
        data.certified_loop_roles = Some(shared_all_material_curve_region_loop_roles(band_count));
        data.certified_loop_fill_rules = Some(Arc::from(vec![FillRule::NonZero; band_count]));
    }
    band = band
        .with_certified_filled_side_is_left(filled_sides_are_left)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
    band.regularized_region_raw(policy)
        .map_err(|error| error.with_operation(CurveOperation2::Offset))
}

fn exact_round_path_cap_band(
    center: &Point2,
    distance: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegionBoundaryLoop2> {
    let positive = center.translated(distance.clone(), Real::zero());
    let negative = center.translated(-distance.clone(), Real::zero());
    let first =
        CircularArc2::try_from_center(positive.clone(), negative.clone(), center.clone(), false)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
    let second = CircularArc2::try_from_center(negative, positive, center.clone(), false)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
    let contour = Contour2::try_new_with_fill_rule(
        vec![Segment2::Arc(first), Segment2::Arc(second)],
        FillRule::NonZero,
    )
    .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
    curve_region_boundary_loop_from_native_material_contour(contour, policy)
        .map_err(|error| error.with_operation(CurveOperation2::Offset))
}

fn exact_path_endpoint_unit_tangent(
    path: &CurvePath2,
    at_start: bool,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<(Real, Real)>> {
    let curve = if at_start {
        &path.curves()[0]
    } else {
        path.curves().last().expect("a curve path is nonempty")
    };
    let parameter = if at_start {
        curve.parameter_domain().start()
    } else {
        curve.parameter_domain().end()
    };
    let Some(parameter) = parameter.scalar() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let max_order = match curve.geometry() {
        None => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        Some(CurveGeometry2::Line(_)) | Some(CurveGeometry2::CircularArc(_)) => 1,
        Some(CurveGeometry2::QuadraticBezier(_))
        | Some(CurveGeometry2::RationalQuadraticBezier(_)) => 2,
        Some(CurveGeometry2::CubicBezier(_)) => 3,
        Some(CurveGeometry2::RationalBezier(curve)) => curve.degree(),
        Some(CurveGeometry2::PolynomialBSpline(curve)) => curve.degree(),
        Some(CurveGeometry2::Nurbs(curve)) => curve.degree(),
    };
    let derivatives = curve
        .derivatives_at_side_with_policy(
            parameter,
            max_order,
            if at_start {
                CurveParameterSide2::Right
            } else {
                CurveParameterSide2::Left
            },
            policy,
        )
        .map_err(|error| error.with_operation(CurveOperation2::Offset))?;
    for (index, derivative) in derivatives.into_iter().enumerate() {
        let length_squared = derivative.dx() * derivative.dx() + derivative.dy() * derivative.dy();
        match real_sign(&length_squared, policy) {
            Some(RealSign::Positive) => {
                let length = length_squared.sqrt().map_err(|cause| {
                    curve_region_edit_error(CurveOperation2::Offset, cause.into())
                })?;
                let orientation = if !at_start && (index + 1).is_multiple_of(2) {
                    -Real::one()
                } else {
                    Real::one()
                };
                return Ok(Classification::Decided((
                    ((derivative.dx() * &orientation) / &length).map_err(|cause| {
                        curve_region_edit_error(CurveOperation2::Offset, cause.into())
                    })?,
                    ((derivative.dy() * orientation) / length).map_err(|cause| {
                        curve_region_edit_error(CurveOperation2::Offset, cause.into())
                    })?,
                )));
            }
            Some(RealSign::Zero) => {}
            Some(RealSign::Negative) => {
                return Err(curve_region_edit_error(
                    CurveOperation2::Offset,
                    CurveError::Topology(
                        "curve endpoint derivative squared norm was certified negative".into(),
                    ),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    Ok(Classification::Uncertain(UncertaintyReason::Boundary))
}

fn exact_line_stroke_band(
    line: LineSeg2,
    distance: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<CurveRegionBoundaryLoop2>> {
    let source = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(line));
    let left = match exact_offset_spans_from_materialized_curve(&source, distance, policy)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
    {
        Classification::Decided(mut spans) if spans.len() == 1 => {
            spans.pop().expect("a line offset produces one exact span")
        }
        Classification::Decided(_) => {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology("a line offset split into multiple source branches".into()),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let right_distance = -distance.clone();
    let right = match exact_offset_spans_from_materialized_curve(&source, &right_distance, policy)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
    {
        Classification::Decided(mut spans) if spans.len() == 1 => {
            spans.pop().expect("a line offset produces one exact span")
        }
        Classification::Decided(_) => {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology("a line offset split into multiple source branches".into()),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let boundary = match exact_offset_span_band_loop(&right, &left, policy)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
    {
        Classification::Decided(boundary) => boundary,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(boundary))
}

fn exact_offset_parallel_endpoint(
    span: &ExactOffsetSpan2,
    at_start: bool,
) -> Option<(&BezierParallel2, Real, bool)> {
    let fragment = if at_start {
        span.fragments.first()
    } else {
        span.fragments.last()
    };
    match fragment? {
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            let parameter = if at_start {
                analytic_parallel_traversal_start(fragment)
            } else {
                analytic_parallel_traversal_end(fragment)
            }
            .scalar()?
            .clone();
            Some((fragment.parallel(), parameter, fragment.is_reversed()))
        }
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let parallel = fragment.analytic_parallel()?;
            let parameter = if at_start != fragment.is_reversed() {
                fragment.range().start()
            } else {
                fragment.range().end()
            }
            .as_bezier_parameter()?
            .scalar()?
            .clone();
            Some((parallel, parameter, fragment.is_reversed()))
        }
        BezierSplitFragment2::Materialized { .. }
        | BezierSplitFragment2::RetainedBezier { .. }
        | BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
    }
}

fn exact_offset_parallel_tangent_contact(
    span: &ExactOffsetSpan2,
    at_start: bool,
    point: &Point2,
) -> Option<crate::rational_bezier::RationalQuadraticParallelCircleContact2> {
    let (parallel, parameter, _) = exact_offset_parallel_endpoint(span, at_start)?;
    Some(
        crate::rational_bezier::RationalQuadraticParallelCircleContact2 {
            parallel: parallel.clone(),
            parameter,
            point: point.clone(),
            eliminant_root_multiplicity: 2,
        },
    )
}

fn exact_offset_parallel_line_tangent_contact(
    span: &ExactOffsetSpan2,
    at_start: bool,
    line_endpoint: BezierEndpoint,
) -> Option<BezierParallelLineTangentContact2> {
    let (parallel, parameter, reversed) = exact_offset_parallel_endpoint(span, at_start)?;
    Some(BezierParallelLineTangentContact2::new(
        parallel.clone(),
        parameter,
        line_endpoint,
        reversed,
    ))
}

fn exact_offset_line_tangent_contact(
    span: &ExactOffsetSpan2,
    at_start: bool,
    point: &Point2,
) -> Option<crate::rational_bezier::RationalQuadraticCircleTangentContact2> {
    let fragment = if at_start {
        span.fragments.first()
    } else {
        span.fragments.last()
    };
    let BezierSplitFragment2::Materialized {
        curve: BezierSubcurve2::Quadratic(curve),
        ..
    } = fragment?
    else {
        return None;
    };
    Some(
        crate::rational_bezier::RationalQuadraticCircleTangentContact2::Line {
            line: curve.retained_exact_line_image()?.clone(),
            point: point.clone(),
        },
    )
}

/// Reuses the retained fillet-circle constructor for a round offset join
/// anchored by one algebraic analytic-parallel endpoint.
///
/// The anchor's original carrier locates the source vertex, while the exact
/// difference between its composed and original parallel distances is the
/// authored round radius. Selected circular and chord companions reuse the
/// certified normal-contact parameters used by fillets. Tangent relations
/// select the chart before the common retained circle publication.
fn append_retained_parallel_round_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    clockwise: bool,
    policy: &CurveContext,
) -> Option<CurveResult<Classification<()>>> {
    let previous_fragment = previous.fragments.last()?;
    let next_fragment = next.fragments.first()?;
    let (
        anchor_is_previous,
        parallel,
        source_parallel,
        parameter,
        selected_source_parameter,
        source_direction,
        companion,
        companion_at_start,
    ) = match (
        &previous.end_tangent,
        previous_fragment,
        &next.start_tangent,
        next_fragment,
    ) {
        (
            Some(CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                parameter,
                selected_source_parameter,
                source_direction,
                ..
            }),
            BezierSplitFragment2::AnalyticParallel(_) | BezierSplitFragment2::SelectedFiber(_),
            _,
            companion @ (BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::AlgebraicChord(_)),
        ) => (
            true,
            parallel,
            source_parallel,
            parameter,
            selected_source_parameter.as_ref(),
            *source_direction,
            companion,
            true,
        ),
        (
            _,
            companion @ (BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::AlgebraicChord(_)),
            Some(CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                parameter,
                selected_source_parameter,
                source_direction,
                ..
            }),
            BezierSplitFragment2::AnalyticParallel(_) | BezierSplitFragment2::SelectedFiber(_),
        ) => (
            false,
            parallel,
            source_parallel,
            parameter,
            selected_source_parameter.as_ref(),
            *source_direction,
            companion,
            false,
        ),
        _ => return None,
    };
    let (anchor_tangent, companion_tangent) = if anchor_is_previous {
        (previous.end_tangent.as_ref()?, next.start_tangent.as_ref()?)
    } else {
        (next.start_tangent.as_ref()?, previous.end_tangent.as_ref()?)
    };
    if matches!(companion, BezierSplitFragment2::AlgebraicChord(_))
        && !matches!(companion_tangent, CurveTangent2::AlgebraicChord(_))
    {
        return None;
    }
    Some((|| {
        let radial_distance = parallel.distance() - source_parallel.distance();
        match is_zero(
            &(&radial_distance * &radial_distance - distance * distance),
            policy,
        ) {
            Some(true) => {}
            Some(false) => {
                return Err(CurveError::Topology(
                    "a retained round-join frame disagreed with its offset distance".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let fillet_clockwise = if anchor_is_previous {
            clockwise
        } else {
            !clockwise
        };
        let fillet = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
            source_parallel.clone(),
            parameter.clone().into(),
            radial_distance,
            fillet_clockwise,
            policy,
        )? {
            Classification::Decided(Some(fillet)) => fillet,
            Classification::Decided(None) => {
                return Err(CurveError::Topology(
                    "a nonzero retained round join collapsed".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let anchor_cross_companion =
            match curve_tangent_cross_sign(anchor_tangent, companion_tangent, policy) {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let sweep_halves = match (fillet_clockwise, anchor_cross_companion) {
            (false, RealSign::Positive) | (true, RealSign::Negative) => 1_u8,
            (false, RealSign::Negative) | (true, RealSign::Positive) => 2_u8,
            (_, RealSign::Zero) => {
                match curve_tangents_are_opposite(anchor_tangent, companion_tangent, policy) {
                    Classification::Decided(true) => 1_u8,
                    Classification::Decided(false) => {
                        return Err(CurveError::Topology(
                            "distinct round-join endpoints retained the same oriented tangent"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        let terminal_circle = if sweep_halves == 2 {
            fillet.complementary_half()
        } else {
            fillet.clone()
        };
        let other_point = if anchor_is_previous {
            next.offset_start.clone()
        } else {
            previous.offset_end.clone()
        };
        let contact_parameter = if anchor_cross_companion == RealSign::Zero {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        } else {
            let terminal_radial_sign = match real_sign(terminal_circle.radial_distance(), policy) {
                Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a retained round terminal circle collapsed".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            let distance_sign = match real_sign(distance, policy) {
                Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Some(RealSign::Zero) => return Ok(Classification::Decided(())),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            let radial_product_sign = exact_sign_product(
                exact_sign_product(terminal_radial_sign, source_direction),
                distance_sign,
            );
            let contact = match companion {
                BezierSplitFragment2::AlgebraicCuspSemicircle(companion) => terminal_circle
                    .certified_selected_circular_tangent_contact_parameter(
                        companion.clone(),
                        companion_at_start,
                        parallel.clone(),
                        selected_source_parameter.map_or_else(
                            || CurveParameter2::from(parameter.clone()),
                            |parameter| CurveParameter2::from_selected_fiber(parameter.clone()),
                        ),
                        source_direction,
                        radial_product_sign,
                        other_point,
                        policy,
                    )?,
                BezierSplitFragment2::AlgebraicChord(_) => {
                    let CurveTangent2::AlgebraicChord(chord) = companion_tangent else {
                        unreachable!("retained chord tangent checked above")
                    };
                    terminal_circle.certified_selected_chord_parallel_normal_contact_parameter(
                        chord.clone(),
                        other_point,
                        distance.clone(),
                        anchor_cross_companion,
                        policy,
                    )?
                }
                _ => unreachable!("retained round-join companion checked above"),
            };
            match contact {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let exact_zero =
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero());
        let exact_one =
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one());
        let mut inserted = if sweep_halves == 1 {
            vec![
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    fillet,
                    exact_zero,
                    contact_parameter,
                    false,
                    policy,
                )
                .with_certified_tangent_endpoints(),
            ]
        } else {
            vec![
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    fillet,
                    exact_zero.clone(),
                    exact_one,
                    false,
                    policy,
                )
                .with_certified_tangent_endpoints(),
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    terminal_circle,
                    exact_zero,
                    contact_parameter,
                    false,
                    policy,
                )
                .with_certified_tangent_endpoints(),
            ]
        };
        if !anchor_is_previous {
            inserted = inserted
                .into_iter()
                .rev()
                .map(|fragment| fragment.reversed())
                .collect();
        }
        fragments.extend(
            inserted
                .into_iter()
                .map(BezierSplitFragment2::AlgebraicCuspSemicircle),
        );
        Ok(Classification::Decided(()))
    })())
}

fn append_exact_round_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    sweep_kind: crate::arc_bezier::ArcSweepKind,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    // Both endpoints were constructed from this vertex with the same signed
    // left-normal distance. The already-certified outer turn therefore fixes
    // both the traversal orientation and the fact that this is the minor arc;
    // do not recompute a potentially wide radical radial cross-product.
    let clockwise = match real_sign(distance, policy) {
        Some(RealSign::Positive) => true,
        Some(RealSign::Negative) => false,
        Some(RealSign::Zero) => return Ok(Classification::Decided(())),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    if sweep_kind == crate::arc_bezier::ArcSweepKind::Minor
        && let Some(result) = append_retained_parallel_round_join(
            fragments, previous, next, distance, clockwise, policy,
        )
    {
        return result;
    }
    if let (Some(previous_offset_end), Some(next_offset_start), Some(center)) = (
        previous.offset_end.coordinates(),
        next.offset_start.coordinates(),
        previous.source_end.coordinates(),
    ) {
        let radius_squared = distance * distance;
        let arc = CircularArc2::new_with_certified_radius_and_sweep(
            previous_offset_end.clone(),
            next_offset_start.clone(),
            center.clone(),
            radius_squared.clone(),
            clockwise,
            sweep_kind,
        );
        let decomposition = match arc.rational_bezier_decomposition_with_policy(policy) {
            Ok(Classification::Decided(decomposition)) => decomposition,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            Err(ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
        };
        let mut parallel_contacts = [
            exact_offset_parallel_tangent_contact(previous, false, previous_offset_end),
            exact_offset_parallel_tangent_contact(next, true, next_offset_start),
        ]
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
        if sweep_kind == crate::arc_bezier::ArcSweepKind::Semicircle
            && parallel_contacts.len() == 2
            && parallel_contacts[0].parallel.source() == parallel_contacts[1].parallel.source()
            && parallel_contacts[0].parameter == parallel_contacts[1].parameter
        {
            // A semicircle between the two limiting sides of one analytic cusp is
            // centered at the cusp parallel. Each neighboring parallel therefore
            // has this circle as its osculating circle, certifying one additional
            // eliminant factor beyond ordinary tangency.
            for contact in &mut parallel_contacts {
                contact.eliminant_root_multiplicity = 3;
            }
        }
        let tangent_contacts = parallel_contacts
            .into_iter()
            .map(crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel)
            .chain(
                [
                    exact_offset_line_tangent_contact(previous, false, previous_offset_end),
                    exact_offset_line_tangent_contact(next, true, next_offset_start),
                ]
                .into_iter()
                .flatten(),
            )
            .collect::<Vec<_>>();
        let circular_conic = Arc::new(crate::rational_bezier::RationalQuadraticCircle2 {
            center: center.clone(),
            radius_squared,
            tangent_contacts: (!tangent_contacts.is_empty()).then(|| Arc::from(tangent_contacts)),
        });
        fragments.extend(decomposition.spans().iter().map(|span| {
            let curve = span.curve().clone().with_retained_conic_provenance(
                span.curve().retained_implicit_quadratic_conic().cloned(),
                Some(Arc::clone(&circular_conic)),
            );
            materialized_offset_fragment(BezierSubcurve2::RationalQuadratic(curve))
        }));
        return Ok(Classification::Decided(()));
    }

    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Minor | crate::arc_bezier::ArcSweepKind::Semicircle
    ) && let (
        Some(CurveTangent2::AlgebraicChord(anchor)),
        Some(CurveTangent2::AlgebraicChord(chord)),
    ) = (previous.end_tangent.as_ref(), next.start_tangent.as_ref())
    {
        return append_selected_chord_pair_round_join(
            fragments, previous, next, distance, clockwise, sweep_kind, anchor, chord, policy,
        );
    }

    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Minor | crate::arc_bezier::ArcSweepKind::Semicircle
    ) && let (Some(previous_tangent), Some(next_tangent)) =
        (previous.end_tangent.as_ref(), next.start_tangent.as_ref())
    {
        if let CurveTangent2::SelectedCircularEndpoint {
            fragment, at_start, ..
        } = previous_tangent
            && let CurveTangent2::AlgebraicChord(chord) = next_tangent
        {
            return append_selected_circle_chord_round_join(
                fragments, previous, next, distance, clockwise, sweep_kind, chord, fragment,
                *at_start, false, policy,
            );
        }
        if let CurveTangent2::AlgebraicChord(chord) = previous_tangent
            && let CurveTangent2::SelectedCircularEndpoint {
                fragment, at_start, ..
            } = next_tangent
        {
            return append_selected_circle_chord_round_join(
                fragments, previous, next, distance, clockwise, sweep_kind, chord, fragment,
                *at_start, true, policy,
            );
        }
    }

    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Minor | crate::arc_bezier::ArcSweepKind::Semicircle
    ) {
        let normal_join = match (previous.end_tangent.as_ref(), next.start_tangent.as_ref()) {
            (
                Some(CurveTangent2::RepresentedDirection(direction)),
                Some(CurveTangent2::AlgebraicChord(chord)),
            ) => Some((direction, chord, false)),
            (
                Some(CurveTangent2::AlgebraicChord(chord)),
                Some(CurveTangent2::RepresentedDirection(direction)),
            ) => Some((direction, chord, true)),
            _ => None,
        };
        if let Some((direction, chord, reversed)) = normal_join {
            let unit_direction = match crate::direction::UnitDirection2::from_direction(direction)?
            {
                Classification::Decided(direction) => direction,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let represented =
                match crate::BezierAlgebraicChord2::from_unit_direction(&unit_direction, policy)? {
                    Classification::Decided(chord) => chord,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let (previous_direction, next_direction) = if reversed {
                (chord, &represented)
            } else {
                (&represented, chord)
            };
            return append_selected_chord_pair_round_join(
                fragments,
                previous,
                next,
                distance,
                clockwise,
                sweep_kind,
                previous_direction,
                next_direction,
                policy,
            );
        }
    }

    // A retained one-field center plus the previous exact unit tangent defines
    // the same selected circular carrier used by analytic cusp and fillet
    // paths. A regular outer corner retains its first half; crossing a local
    // collapse radius retains the complete semicircle.
    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Minor | crate::arc_bezier::ArcSweepKind::Semicircle
    ) && let Some(CurveTangent2::RepresentedDirection((tangent_x, tangent_y))) =
        previous.end_tangent.as_ref()
    {
        let unit_residual = tangent_x * tangent_x + tangent_y * tangent_y - Real::one();
        if is_zero(&unit_residual, &CurveContext::STRICT) == Some(true) {
            let semicircle = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_certified_unit_normal(
                &previous.source_end,
                (-tangent_y.clone(), tangent_x.clone()),
                distance.clone(),
                clockwise,
                policy,
            )? {
                Classification::Decided(Some(semicircle)) => semicircle,
                Classification::Decided(None) => return Ok(Classification::Decided(())),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let quadrant_minor = match next.start_tangent.as_ref() {
                Some(CurveTangent2::RepresentedDirection((next_x, next_y))) => {
                    is_zero(
                        &(tangent_x * next_x + tangent_y * next_y),
                        &CurveContext::STRICT,
                    ) == Some(true)
                }
                _ => false,
            };
            let end_parameter = match sweep_kind {
                crate::arc_bezier::ArcSweepKind::Minor if quadrant_minor => {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                        (Real::one() / Real::from(2_i8))?,
                    )
                }
                crate::arc_bezier::ArcSweepKind::Minor => {
                    // The selected carrier starts at the previous offset
                    // endpoint, but a general corner need not turn through a
                    // quadrant. Intersect its exact endpoint chord with that
                    // carrier and reuse the existing circle/chord authority
                    // to name the actual minor-arc cut.
                    let (endpoint_chord, endpoint_chord_parameter) = if let Some(
                        BezierSplitFragment2::AlgebraicChord(chord),
                    ) = next.fragments.first()
                    {
                        (chord.clone(), chord.start_parameter())
                    } else {
                        let chord = match crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                                previous.offset_end.clone(),
                                next.offset_start.clone(),
                                policy,
                            )? {
                                Classification::Decided(chord) => chord,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                        let endpoint = chord.end_parameter();
                        (chord, endpoint)
                    };
                    let contacts = match semicircle.chord_intersections(&endpoint_chord, policy)? {
                        Classification::Decided(contacts) if contacts.is_empty() => {
                            return Err(CurveError::Topology(
                                "an authored round-join endpoint missed its selected circle".into(),
                            ));
                        }
                        Classification::Decided(contacts) => contacts,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let mut endpoint_parameter = None;
                    let mut uncertainty = None;
                    for contact in contacts {
                        match contact
                            .chord_parameter
                            .cmp_by_refinement(&endpoint_chord_parameter, policy)?
                        {
                            Classification::Decided(std::cmp::Ordering::Equal) => {
                                if endpoint_parameter.replace(contact.cusp_parameter).is_some() {
                                    return Err(CurveError::Topology(
                                        "an authored round-join endpoint had duplicate selected-circle contacts"
                                            .into(),
                                    ));
                                }
                            }
                            Classification::Decided(_) => {}
                            Classification::Uncertain(reason) => {
                                uncertainty.get_or_insert(reason);
                            }
                        }
                    }
                    match endpoint_parameter {
                        Some(parameter) => parameter,
                        None => {
                            if let Some(reason) = uncertainty {
                                return Ok(Classification::Uncertain(reason));
                            }
                            return Err(CurveError::Topology(
                                "an authored round-join endpoint was absent from its selected-circle contact set"
                                    .into(),
                            ));
                        }
                    }
                }
                crate::arc_bezier::ArcSweepKind::Semicircle => {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
                }
                crate::arc_bezier::ArcSweepKind::Major
                | crate::arc_bezier::ArcSweepKind::FullCircle => {
                    unreachable!("the retained unit-frame round path admits at most one semicircle")
                }
            };
            let fragment = match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                semicircle,
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                end_parameter,
                false,
                policy,
            )? {
                Classification::Decided(fragment) => fragment,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for (at_start, expected) in [(true, &previous.offset_end), (false, &next.offset_start)]
            {
                match fragment.certify_and_cache_authored_endpoint(at_start, expected, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Err(CurveError::Topology(
                            "retained selected semicircle join missed its certified endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            fragments.push(BezierSplitFragment2::AlgebraicCuspSemicircle(
                fragment.with_certified_tangent_endpoints(),
            ));
            return Ok(Classification::Decided(()));
        }
    }
    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Minor | crate::arc_bezier::ArcSweepKind::Semicircle
    ) && let Some((previous_tangent, next_tangent)) = previous
        .end_tangent
        .as_ref()
        .zip(next.start_tangent.as_ref())
        && let (Some(previous_support), Some(next_support)) = (
            exact_offset_retained_tangent_support(previous_tangent, &previous.offset_end, policy),
            exact_offset_retained_tangent_support(next_tangent, &next.offset_start, policy),
        )
    {
        let anchor = match previous_support? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let companion = match next_support? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        // Both supports retain their oriented endpoint directions. The common
        // chord-normal frame also covers parallel cusp joins whose exact
        // endpoints remain procedural, without a separate circular carrier.
        return append_selected_chord_pair_round_join(
            fragments, previous, next, distance, clockwise, sweep_kind, &anchor, &companion, policy,
        );
    }
    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
}

#[allow(clippy::too_many_arguments)]
fn append_selected_circle_chord_round_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    clockwise: bool,
    sweep_kind: crate::arc_bezier::ArcSweepKind,
    chord: &crate::BezierAlgebraicChord2,
    selected_circle: &crate::BezierAlgebraicCuspSemicircleFragment2,
    selected_at_start: bool,
    anchor_is_previous: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    // Use the algebraic chord as the local unit frame. Its offset endpoint is
    // parameter zero on the join circle; reversing the selected half when the
    // chord follows the circular span restores the requested boundary order.
    // The other authored endpoint is then selected by the same authoritative
    // selected-circle/chord kernel used everywhere else.
    let (other_point, fillet_clockwise, reversed) = if anchor_is_previous {
        (&next.offset_start, clockwise, false)
    } else {
        (&previous.offset_end, !clockwise, true)
    };
    let center = if anchor_is_previous {
        chord.end().clone()
    } else {
        chord.start().clone()
    };
    let fillet = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
        center,
        chord.clone(),
        distance.clone(),
        fillet_clockwise,
        policy,
    )? {
        Classification::Decided(Some(fillet)) => fillet,
        Classification::Decided(None) => return Ok(Classification::Decided(())),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_parameter = match sweep_kind {
        crate::arc_bezier::ArcSweepKind::Semicircle => {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        }
        crate::arc_bezier::ArcSweepKind::Minor => {
            let tangent = match selected_circle.endpoint_tangent_chord(selected_at_start, policy)? {
                Classification::Decided(Some(tangent)) => tangent,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match fillet.certified_chord_normal_contact_parameter(
                crate::bezier_offset::BezierSelectedChordNormalAnchor2::RetainedChord(
                    chord.clone(),
                ),
                tangent,
                other_point.clone(),
                distance.clone(),
                true,
                policy,
            )? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        crate::arc_bezier::ArcSweepKind::Major | crate::arc_bezier::ArcSweepKind::FullCircle => {
            unreachable!("an exact offset round join admits at most one semicircle")
        }
    };
    let fragment = match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
        fillet,
        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
        end_parameter,
        reversed,
        policy,
    )? {
        Classification::Decided(fragment) => fragment,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    for (at_start, expected) in [(true, &previous.offset_end), (false, &next.offset_start)] {
        if at_start == anchor_is_previous {
            // Parameter zero was constructed from this exact chord endpoint
            // and its signed left-normal offset. That shared carrier is the
            // incidence certificate; comparing its independently wrapped
            // center point would only rebuild the same selected fields.
            continue;
        }
        match fragment.certify_and_cache_authored_endpoint(at_start, expected, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Err(CurveError::Topology(
                    "a retained selected-circle/chord round join missed its certified endpoint"
                        .into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-join",
        "selected-circle-chord-round",
    );
    fragments.push(BezierSplitFragment2::AlgebraicCuspSemicircle(
        fragment.with_certified_tangent_endpoints(),
    ));
    Ok(Classification::Decided(()))
}

#[allow(clippy::too_many_arguments)]
fn append_selected_chord_pair_round_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    clockwise: bool,
    sweep_kind: crate::arc_bezier::ArcSweepKind,
    anchor: &crate::BezierAlgebraicChord2,
    chord: &crate::BezierAlgebraicChord2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let represented_unit_tangent = anchor.certified_unit_tangent().or_else(|| {
        anchor
            .certified_axis_direction()
            .map(BezierAlgebraicChordAxisDirection2::unit_tangent)
    });
    let chord_frame = || {
        crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
            previous.source_end.clone(),
            anchor.clone(),
            distance.clone(),
            clockwise,
            policy,
        )
    };
    let (selected, represented_frame) = if let Some((tangent_x, tangent_y)) =
        represented_unit_tangent.as_ref()
    {
        let represented = crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_certified_unit_normal(
            &previous.source_end,
            (-tangent_y.clone(), tangent_x.clone()),
            distance.clone(),
            clockwise,
            policy,
        )?;
        match represented {
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                // A represented circle frame owns one algebraic center field.
                // Exact and other retained centers instead share the anchor's
                // general chord-normal frame without projecting coordinates.
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-join",
                    "selected-chord-pair-round-chord-frame-fallback",
                );
                (chord_frame()?, false)
            }
            represented => (represented, true),
        }
    } else {
        (chord_frame()?, false)
    };
    let semicircle = match selected {
        Classification::Decided(Some(semicircle)) => semicircle,
        Classification::Decided(None) => return Ok(Classification::Decided(())),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end = match sweep_kind {
        crate::arc_bezier::ArcSweepKind::Minor => {
            let selected = match (represented_frame, represented_unit_tangent) {
                (true, Some(anchor_tangent)) => semicircle
                    .certified_chord_normal_contact_parameter(
                        crate::bezier_offset::BezierSelectedChordNormalAnchor2::Represented(
                            anchor_tangent,
                        ),
                        chord.clone(),
                        next.offset_start.clone(),
                        distance.clone(),
                        false,
                        policy,
                    )?,
                (false, _) => semicircle.certified_chord_normal_contact_parameter(
                    crate::bezier_offset::BezierSelectedChordNormalAnchor2::RetainedChord(
                        anchor.clone(),
                    ),
                    chord.clone(),
                    next.offset_start.clone(),
                    distance.clone(),
                    false,
                    policy,
                )?,
                (true, None) => unreachable!("a represented frame has a represented tangent"),
            };
            match selected {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        crate::arc_bezier::ArcSweepKind::Semicircle => {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        }
        crate::arc_bezier::ArcSweepKind::Major | crate::arc_bezier::ArcSweepKind::FullCircle => {
            unreachable!("an exact offset round join admits at most one semicircle")
        }
    };
    let fragment = match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
        semicircle,
        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
        end,
        false,
        policy,
    )? {
        Classification::Decided(fragment) => fragment,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    for (at_start, expected) in [(true, &previous.offset_end), (false, &next.offset_start)] {
        match fragment.certify_and_cache_authored_endpoint(at_start, expected, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Err(CurveError::Topology(
                    "retained chord-pair round join missed its certified endpoint".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-join",
        "selected-chord-pair-round",
    );
    fragments.push(BezierSplitFragment2::AlgebraicCuspSemicircle(
        fragment.with_certified_tangent_endpoints(),
    ));
    Ok(Classification::Decided(()))
}

fn append_exact_line_join_with_parallel_tangencies(
    fragments: &mut Vec<BezierSplitFragment2>,
    from: &Point2,
    to: &Point2,
    parallel_tangent_contacts: Vec<BezierParallelLineTangentContact2>,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    match is_zero(&from.distance_squared(to), policy) {
        Some(true) => Ok(Classification::Decided(())),
        Some(false) => {
            let line = LineSeg2::try_new(from.clone(), to.clone())?;
            let curve = if parallel_tangent_contacts.is_empty() {
                QuadraticBezier2::from_line_segment(line)
            } else {
                QuadraticBezier2::from_line_segment_with_parallel_tangent_contacts(
                    line,
                    parallel_tangent_contacts,
                )
            };
            fragments.push(materialized_offset_fragment(BezierSubcurve2::Quadratic(
                curve,
            )));
            Ok(Classification::Decided(()))
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn append_exact_miter_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    limit: Option<&Real>,
    turn_sign: RealSign,
    distance_sign: RealSign,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let represented_vector_frame = matches!(
        (&previous.end_tangent, &next.start_tangent),
        (
            Some(CurveTangent2::RepresentedDirection(_)),
            Some(CurveTangent2::RepresentedDirection(_))
        )
    ) && previous.offset_end.coordinates().is_some()
        && next.offset_start.coordinates().is_some()
        && previous.source_end.coordinates().is_some();
    if !represented_vector_frame
        && let Some(result) = append_retained_support_miter_join(
            fragments,
            previous,
            next,
            distance,
            limit,
            turn_sign,
            distance_sign,
            policy,
        )
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-miter",
            "retained-support",
        );
        return result;
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-miter",
        "represented-vector-fallback",
    );
    let (
        Some(CurveTangent2::RepresentedDirection(previous_tangent)),
        Some(CurveTangent2::RepresentedDirection(next_tangent)),
    ) = (&previous.end_tangent, &next.start_tangent)
    else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-miter",
            "non-vector-tangent",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let (Some(previous_offset_end), Some(next_offset_start), Some(source_vertex)) = (
        previous.offset_end.coordinates(),
        next.offset_start.coordinates(),
        previous.source_end.coordinates(),
    ) else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-miter",
            "non-represented-point",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let denominator = if offset_vectors_are_structurally_opposite(previous_tangent, next_tangent) {
        Real::zero()
    } else {
        offset_vector_cross(previous_tangent, next_tangent)
    };
    let denominator_sign = real_sign(&denominator, policy);
    let Some(RealSign::Positive | RealSign::Negative) = denominator_sign else {
        return match denominator_sign {
            Some(RealSign::Zero) => append_exact_line_join_with_parallel_tangencies(
                fragments,
                previous_offset_end,
                next_offset_start,
                // Equal tangents make the endpoints coincide; opposite
                // tangents connect across their normals. Neither surviving
                // line is an analytic-parallel tangent leg.
                Vec::new(),
                policy,
            ),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            Some(RealSign::Positive | RealSign::Negative) => unreachable!(),
        };
    };
    let denominator_reciprocal = denominator.inverse_ref_assuming_nonzero()?;
    let delta = next_offset_start.delta_from(previous_offset_end);
    let numerator = &delta.0 * &next_tangent.1 - &delta.1 * &next_tangent.0;
    let parameter = numerator * denominator_reciprocal;
    let miter = previous_offset_end.translated(
        &previous_tangent.0 * &parameter,
        &previous_tangent.1 * parameter,
    );
    if let Some(limit) = limit {
        let miter_distance_squared = miter.distance_squared(source_vertex);
        let maximum_squared = distance * distance * limit * limit;
        match compare_reals(&miter_distance_squared, &maximum_squared, policy) {
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {}
            Some(std::cmp::Ordering::Greater) => {
                return append_exact_line_join_with_parallel_tangencies(
                    fragments,
                    previous_offset_end,
                    next_offset_start,
                    // A rejected miter becomes the transverse bevel joining
                    // the two offset endpoints, not either tangent ray.
                    Vec::new(),
                    policy,
                );
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }
    let previous_contacts =
        exact_offset_parallel_line_tangent_contact(previous, false, BezierEndpoint::Start)
            .into_iter()
            .collect::<Vec<_>>();
    match append_exact_line_join_with_parallel_tangencies(
        fragments,
        previous_offset_end,
        &miter,
        previous_contacts,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let next_contacts = exact_offset_parallel_line_tangent_contact(next, true, BezierEndpoint::End)
        .into_iter()
        .collect::<Vec<_>>();
    append_exact_line_join_with_parallel_tangencies(
        fragments,
        &miter,
        next_offset_start,
        next_contacts,
        policy,
    )
}

fn exact_retained_parallel_represented_tangent(
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    source_direction: RealSign,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<(Real, Real)>>> {
    let Some(parameter) = parameter.scalar() else {
        return Ok(Classification::Decided(None));
    };
    let tangent = match parallel.source_tangent_at(parameter, policy)? {
        Classification::Decided(tangent) => tangent,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(Some(match source_direction {
        RealSign::Positive => tangent,
        RealSign::Negative => (-tangent.0, -tangent.1),
        RealSign::Zero => {
            return Err(CurveError::Topology(
                "a retained parallel tangent had zero traversal direction".into(),
            ));
        }
    })))
}

fn exact_offset_retained_tangent_support(
    tangent: &CurveTangent2,
    endpoint: &CurvePoint2,
    policy: &CurveContext,
) -> Option<CurveResult<Classification<crate::BezierAlgebraicChord2>>> {
    match tangent {
        CurveTangent2::RepresentedDirection(vector) => Some((|| {
            let displaced = match crate::BezierAlgebraicChord2::translated_endpoint(
                endpoint, &vector.0, &vector.1, policy,
            )? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                endpoint.clone(),
                displaced,
                policy,
            )
        })()),
        CurveTangent2::RetainedParallel {
            parallel,
            parameter,
            source_direction,
            ..
        } => Some((|| {
            match exact_retained_parallel_represented_tangent(
                parallel,
                parameter,
                *source_direction,
                policy,
            )? {
                Classification::Decided(Some(vector)) => {
                    let displaced = match crate::BezierAlgebraicChord2::translated_endpoint(
                        endpoint, &vector.0, &vector.1, policy,
                    )? {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    return crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                        endpoint.clone(),
                        displaced,
                        policy,
                    );
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let tangent_distance = Real::from(match source_direction {
                RealSign::Positive => 1_i8,
                RealSign::Negative => -1_i8,
                RealSign::Zero => {
                    return Err(CurveError::Topology(
                        "a retained miter tangent had zero traversal direction".into(),
                    ));
                }
            });
            let displaced = CurvePoint2::from(
                crate::BezierAnalyticParallelPoint2::new_with_tangent_distance(
                    parallel.clone(),
                    parameter.clone(),
                    tangent_distance,
                    policy,
                ),
            );
            crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                endpoint.clone(),
                displaced,
                policy,
            )
        })()),
        CurveTangent2::AlgebraicChord(chord) => Some(Ok(Classification::Decided(chord.clone()))),
        CurveTangent2::SelectedCircularEndpoint {
            fragment, at_start, ..
        }
        | CurveTangent2::ChordContact {
            fragment, at_start, ..
        } => Some((|| {
            let displaced = match fragment.endpoint_tangent_support_point(*at_start, policy)? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                endpoint.clone(),
                displaced,
                policy,
            )
        })()),
        CurveTangent2::CircularPoint { .. } => None,
    }
}

fn exact_offset_span_retained_tangent_support(
    span: &ExactOffsetSpan2,
    at_start: bool,
    policy: &CurveContext,
) -> Option<CurveResult<Classification<(crate::BezierAlgebraicChord2, bool)>>> {
    let fragment = if at_start {
        span.fragments.first()
    } else {
        span.fragments.last()
    };
    if let Some(BezierSplitFragment2::AlgebraicChord(chord)) = fragment {
        // The offset span itself is the authoritative finite subset of this
        // tangent support. Reuse it so a later miter intersection retains the
        // same support identity used by Boolean regularization.
        return Some(Ok(Classification::Decided((chord.clone(), !at_start))));
    }
    // A carrier may collapse its finite offset image while still retaining an
    // authored endpoint tangent for the adjacent inner join.  The tangent is
    // the supporting-line authority; absence of an emitted finite fragment
    // must not suppress it before the complete retained miter path runs.
    let tangent = if at_start {
        span.start_tangent.as_ref()
    } else {
        span.end_tangent.as_ref()
    }?;
    let endpoint = if at_start {
        &span.offset_start
    } else {
        &span.offset_end
    };
    exact_offset_retained_tangent_support(tangent, endpoint, policy)
        .map(|result| result.map(|classification| classification.map(|chord| (chord, false))))
}

fn append_retained_support_miter_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    limit: Option<&Real>,
    turn_sign: RealSign,
    distance_sign: RealSign,
    policy: &CurveContext,
) -> Option<CurveResult<Classification<()>>> {
    let previous_support = exact_offset_span_retained_tangent_support(previous, false, policy)?;
    let next_support = exact_offset_span_retained_tangent_support(next, true, policy)?;
    Some((|| {
        let previous_support = match previous_support {
            Err(error) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-error",
                    "previous-support",
                );
                return Err(error);
            }
            Ok(classification) => match classification {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-miter-blocker",
                        "previous-support",
                    );
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        let next_support = match next_support {
            Err(error) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-error",
                    "next-support",
                );
                return Err(error);
            }
            Ok(classification) => match classification {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-miter-blocker",
                        "next-support",
                    );
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        let (previous_support, previous_anchor_at_end) = previous_support;
        let (next_support, next_anchor_at_end) = next_support;
        let displacement_sign = exact_sign_product(turn_sign, distance_sign);
        let first_order = match displacement_sign {
            RealSign::Positive => std::cmp::Ordering::Less,
            RealSign::Negative => std::cmp::Ordering::Greater,
            RealSign::Zero => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
        };
        let second_order = first_order.reverse();
        let support_intersection = match previous_support
            .supporting_line_intersection_with_certified_anchor_orders(
                &next_support,
                previous_anchor_at_end,
                first_order,
                next_anchor_at_end,
                second_order,
                policy,
            ) {
            Ok(intersection) => intersection,
            Err(error) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-error",
                    "support-intersection",
                );
                return Err(error);
            }
        };
        let miter = match support_intersection {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter",
                    "parallel-supports",
                );
                return append_exact_algebraic_line_join(
                    fragments,
                    &previous.offset_end,
                    &next.offset_start,
                    None,
                    None,
                    true,
                    [false; 2],
                    policy,
                );
            }
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-blocker",
                    "support-intersection",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let Some(limit) = limit {
            let maximum_squared = distance * distance * limit * limit;
            let strict_policy = policy.strict_counterpart();
            let within_limit = match crate::bezier_offset::algebraic_point_distance_squared_at_most(
                &miter,
                &previous.source_end,
                &maximum_squared,
                &strict_policy,
            ) {
                Classification::Decided(within_limit) => Classification::Decided(within_limit),
                Classification::Uncertain(_) => {
                    if let (
                        Some(CurveTangent2::RepresentedDirection(previous_tangent)),
                        Some(CurveTangent2::RepresentedDirection(next_tangent)),
                    ) = (&previous.end_tangent, &next.start_tangent)
                    {
                        // For unit traversal tangents u and v, the squared distance
                        // from their shared source vertex to the equal-distance
                        // offset-line intersection is d²·2/(1 + u·v).  Comparing
                        // 2 <= limit²·(1 + u·v) therefore decides the authored miter
                        // limit without adjoining either retained endpoint field.
                        let previous_norm_squared = &previous_tangent.0 * &previous_tangent.0
                            + &previous_tangent.1 * &previous_tangent.1;
                        let next_norm_squared =
                            &next_tangent.0 * &next_tangent.0 + &next_tangent.1 * &next_tangent.1;
                        match (
                            is_zero(&(previous_norm_squared - Real::one()), &strict_policy),
                            is_zero(&(next_norm_squared - Real::one()), &strict_policy),
                        ) {
                            (Some(true), Some(true)) => {
                                let one_plus_dot = Real::one()
                                    + &previous_tangent.0 * &next_tangent.0
                                    + &previous_tangent.1 * &next_tangent.1;
                                match real_sign(&one_plus_dot, &strict_policy) {
                                    Some(RealSign::Positive) => {
                                        let threshold = limit * limit * one_plus_dot;
                                        match compare_reals(
                                            &Real::from(2_i8),
                                            &threshold,
                                            &strict_policy,
                                        ) {
                                            Some(
                                                std::cmp::Ordering::Less
                                                | std::cmp::Ordering::Equal,
                                            ) => Classification::Decided(true),
                                            Some(std::cmp::Ordering::Greater) => {
                                                Classification::Decided(false)
                                            }
                                            None => Classification::Uncertain(
                                                UncertaintyReason::Ordering,
                                            ),
                                        }
                                    }
                                    Some(RealSign::Zero) => {
                                        Classification::Uncertain(UncertaintyReason::Boundary)
                                    }
                                    Some(RealSign::Negative) => {
                                        return Err(CurveError::Topology(
                                            "unit miter tangents had a dot product below minus one"
                                                .into(),
                                        ));
                                    }
                                    None => Classification::Uncertain(UncertaintyReason::RealSign),
                                }
                            }
                            (Some(false), _) | (_, Some(false)) => {
                                Classification::Uncertain(UncertaintyReason::Unsupported)
                            }
                            _ => Classification::Uncertain(UncertaintyReason::RealSign),
                        }
                    } else {
                        Classification::Uncertain(UncertaintyReason::Ordering)
                    }
                }
            };
            let within_limit = match within_limit {
                Classification::Uncertain(reason) if policy.permits_approximate_512() => {
                    match crate::bezier_offset::algebraic_point_distance_squared_at_most(
                        &miter,
                        &previous.source_end,
                        &maximum_squared,
                        policy,
                    ) {
                        decided @ Classification::Decided(_) => decided,
                        Classification::Uncertain(_) => Classification::Uncertain(reason),
                    }
                }
                classification => classification,
            };
            match within_limit {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return append_exact_algebraic_line_join(
                        fragments,
                        &previous.offset_end,
                        &next.offset_start,
                        None,
                        None,
                        true,
                        [
                            previous
                                .end_tangent
                                .as_ref()
                                .is_some_and(exact_offset_tangent_is_selected_circle),
                            next.start_tangent
                                .as_ref()
                                .is_some_and(exact_offset_tangent_is_selected_circle),
                        ],
                        policy,
                    );
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let previous_contacts: Vec<_> =
            exact_offset_parallel_line_tangent_contact(previous, false, BezierEndpoint::Start)
                .into_iter()
                .collect();
        let previous_leg = match append_retained_support_miter_leg(
            fragments,
            &previous_support,
            previous.offset_end.clone(),
            miter.clone(),
            previous_contacts,
            policy,
        ) {
            Ok(classification) => classification,
            Err(error) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-error",
                    "previous-leg",
                );
                return Err(error);
            }
        };
        match previous_leg {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-blocker",
                    "previous-leg",
                );
                return Ok(Classification::Uncertain(reason));
            }
        }
        let next_contacts: Vec<_> =
            exact_offset_parallel_line_tangent_contact(next, true, BezierEndpoint::End)
                .into_iter()
                .collect();
        let next_leg = match append_retained_support_miter_leg(
            fragments,
            &next_support,
            miter,
            next.offset_start.clone(),
            next_contacts,
            policy,
        ) {
            Ok(classification) => classification,
            Err(error) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-error",
                    "next-leg",
                );
                return Err(error);
            }
        };
        #[cfg(feature = "dispatch-trace")]
        if matches!(next_leg, Classification::Uncertain(_)) {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-miter-blocker",
                "next-leg",
            );
        }
        Ok(next_leg)
    })())
}

fn append_retained_support_miter_leg(
    fragments: &mut Vec<BezierSplitFragment2>,
    support: &crate::BezierAlgebraicChord2,
    from: CurvePoint2,
    to: CurvePoint2,
    parallel_tangent_contacts: Vec<BezierParallelLineTangentContact2>,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    match support.chord_between_certified_support_points(from, to, policy)? {
        Classification::Decided(Some(chord)) => {
            let chord = chord.with_parallel_tangent_contacts(parallel_tangent_contacts);
            fragments.push(retained_chord_fragment(chord));
            Ok(Classification::Decided(()))
        }
        Classification::Decided(None) => Ok(Classification::Decided(())),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn offset_vector_cross(first: &(Real, Real), second: &(Real, Real)) -> Real {
    &first.0 * &second.1 - &first.1 * &second.0
}

const fn exact_sign_product(first: RealSign, second: RealSign) -> RealSign {
    match (first, second) {
        (RealSign::Zero, _) | (_, RealSign::Zero) => RealSign::Zero,
        (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
            RealSign::Positive
        }
        (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive) => {
            RealSign::Negative
        }
    }
}

const fn exact_sign_reverse(sign: RealSign) -> RealSign {
    match sign {
        RealSign::Negative => RealSign::Positive,
        RealSign::Zero => RealSign::Zero,
        RealSign::Positive => RealSign::Negative,
    }
}

fn exact_circular_tangent_cross_vector(
    point: &CurvePoint2,
    circle: &crate::rational_bezier::RationalQuadraticCircle2,
    clockwise: bool,
    vector: &(Real, Real),
    policy: &CurveContext,
) -> Classification<RealSign> {
    {
        match algebraic_chord_point_linear_order_to_exact(
            point,
            &circle.center,
            &vector.0,
            &vector.1,
            policy,
        ) {
            Ok(Classification::Decided(order)) => {
                let radial_projection_sign = match order {
                    std::cmp::Ordering::Less => RealSign::Negative,
                    std::cmp::Ordering::Equal => RealSign::Zero,
                    std::cmp::Ordering::Greater => RealSign::Positive,
                };
                let orientation = if clockwise {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                };
                Classification::Decided(exact_sign_product(radial_projection_sign, orientation))
            }
            Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
            Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
        }
    }
}

fn exact_algebraic_chord_parallel_factor(
    reference: &crate::BezierAlgebraicChord2,
    candidate: &crate::BezierAlgebraicChord2,
    policy: &CurveContext,
) -> Classification<RealSign> {
    if let (Some(first), Some(second)) = (
        reference.certified_unit_tangent(),
        candidate.certified_unit_tangent(),
    ) {
        let cross = Real::diff_of_products(&first.0, &second.1, &first.1, &second.0);
        // The represented-unit-vector comparison is only a fast path.  Keep
        // it certified so an unresolved symbolic cancellation can fall
        // through to the retained chord relation below instead of consuming
        // APPROXIMATE_512 before that exact certificate is consulted.
        match policy.strict_predicate_pass(|| real_sign(&cross, policy)) {
            Some(RealSign::Zero) => {
                let dot = Real::dot2_refs([&first.0, &first.1], [&second.0, &second.1]);
                match policy.strict_predicate_pass(|| real_sign(&dot, policy)) {
                    Some(sign @ (RealSign::Negative | RealSign::Positive)) => {
                        return Classification::Decided(sign);
                    }
                    Some(RealSign::Zero) => {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                    None => {}
                }
            }
            Some(RealSign::Negative | RealSign::Positive) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            None => {}
        }
    }
    match reference.tangent_cross_sign(candidate, policy) {
        Ok(Classification::Decided(RealSign::Zero)) => {}
        Ok(Classification::Decided(RealSign::Negative | RealSign::Positive)) => {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    }
    match reference.tangent_dot_sign(candidate, policy) {
        Ok(Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive))) => {
            Classification::Decided(sign)
        }
        Ok(Classification::Decided(RealSign::Zero)) => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn exact_algebraic_chord_vector_factor(
    reference: &crate::BezierAlgebraicChord2,
    candidate: &(Real, Real),
    policy: &CurveContext,
) -> Classification<RealSign> {
    match reference.tangent_cross_vector_sign(candidate, policy) {
        Ok(Classification::Decided(RealSign::Zero)) => {}
        Ok(Classification::Decided(RealSign::Negative | RealSign::Positive)) => {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    }
    match reference.tangent_dot_vector_sign(candidate, policy) {
        Ok(Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive))) => {
            Classification::Decided(sign)
        }
        Ok(Classification::Decided(RealSign::Zero)) => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn exact_algebraic_chord_retained_parallel_relation(
    chord: &crate::BezierAlgebraicChord2,
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    source_direction: RealSign,
    cross: bool,
    policy: &CurveContext,
) -> Classification<RealSign> {
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        if cross {
            "curve-region-exact-offset-tangent-cross"
        } else {
            "curve-region-exact-offset-tangent-dot"
        },
        "algebraic-chord-retained-parallel",
    );
    let represented = chord.certified_unit_tangent().or_else(|| {
        chord
            .certified_axis_direction()
            .map(BezierAlgebraicChordAxisDirection2::unit_tangent)
    });
    let relation = if let Some(tangent) = represented {
        match parallel.vector_tangent_cross_and_dot_signs(
            &parameter.clone().into(),
            &tangent.0,
            &tangent.1,
            policy,
        ) {
            Ok(Classification::Decided((cross_sign, dot_sign))) => {
                if cross {
                    cross_sign
                } else {
                    dot_sign
                }
            }
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        }
    } else {
        let zero = Real::zero();
        let one = Real::one();
        match chord.tangent_cross_dot_parallel_linear_combination_sign(
            parallel,
            &parameter.clone().into(),
            if cross { &one } else { &zero },
            if cross { &zero } else { &one },
            policy,
        ) {
            Ok(Classification::Decided(relation)) => relation,
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        }
    };
    let parallel_scale =
        match parallel.parallel_derivative_scale_sign(&parameter.clone().into(), policy) {
            Ok(Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative))) => sign,
            Ok(Classification::Decided(RealSign::Zero)) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
    Classification::Decided(exact_sign_product(
        relation,
        exact_sign_product(source_direction, parallel_scale),
    ))
}

fn exact_retained_parallel_tangent_cross_and_dot_vector(
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    source_direction: RealSign,
    vector: &(Real, Real),
    policy: &CurveContext,
) -> Classification<(RealSign, RealSign)> {
    match exact_retained_parallel_represented_tangent(parallel, parameter, source_direction, policy)
    {
        Ok(Classification::Decided(Some(tangent))) => {
            let cross = offset_vector_cross(&tangent, vector);
            let dot = &tangent.0 * &vector.0 + &tangent.1 * &vector.1;
            return match (real_sign(&cross, policy), real_sign(&dot, policy)) {
                (Some(cross), Some(dot)) => Classification::Decided((cross, dot)),
                _ => Classification::Uncertain(UncertaintyReason::RealSign),
            };
        }
        Ok(Classification::Decided(None)) => {}
        Ok(Classification::Uncertain(reason)) => {
            return Classification::Uncertain(reason);
        }
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    }
    let (vector_cross_parallel, vector_dot_parallel) = match parallel
        .vector_tangent_cross_and_dot_signs(&parameter.clone().into(), &vector.0, &vector.1, policy)
    {
        Ok(Classification::Decided(signs)) => signs,
        Ok(Classification::Uncertain(reason)) => {
            return Classification::Uncertain(reason);
        }
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let parallel_scale =
        match parallel.parallel_derivative_scale_sign(&parameter.clone().into(), policy) {
            Ok(Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative))) => sign,
            Ok(Classification::Decided(RealSign::Zero)) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
    let factor = exact_sign_product(source_direction, parallel_scale);
    Classification::Decided((
        exact_sign_reverse(exact_sign_product(vector_cross_parallel, factor)),
        exact_sign_product(vector_dot_parallel, factor),
    ))
}

#[allow(clippy::too_many_arguments)]
fn exact_retained_parallel_tangent_pair_cross_and_dot(
    first_parallel: &BezierParallel2,
    first_parameter: &BezierParameter2,
    first_direction: RealSign,
    second_parallel: &BezierParallel2,
    second_parameter: &BezierParameter2,
    second_direction: RealSign,
    policy: &CurveContext,
) -> Classification<(RealSign, RealSign)> {
    let first_represented = match exact_retained_parallel_represented_tangent(
        first_parallel,
        first_parameter,
        first_direction,
        policy,
    ) {
        Ok(Classification::Decided(tangent)) => tangent,
        Ok(Classification::Uncertain(reason)) => {
            return Classification::Uncertain(reason);
        }
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let second_represented = match exact_retained_parallel_represented_tangent(
        second_parallel,
        second_parameter,
        second_direction,
        policy,
    ) {
        Ok(Classification::Decided(tangent)) => tangent,
        Ok(Classification::Uncertain(reason)) => {
            return Classification::Uncertain(reason);
        }
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    if let (Some(first), Some(second)) = (first_represented, second_represented) {
        let cross = offset_vector_cross(&first, &second);
        let dot = &first.0 * &second.0 + &first.1 * &second.1;
        return match (real_sign(&cross, policy), real_sign(&dot, policy)) {
            (Some(cross), Some(dot)) => Classification::Decided((cross, dot)),
            _ => Classification::Uncertain(UncertaintyReason::RealSign),
        };
    }
    match first_parallel.source_tangent_pair_cross_and_dot_signs(
        first_parameter,
        second_parallel,
        second_parameter,
        policy,
    ) {
        Ok(Classification::Decided((cross, dot))) => {
            let factor = exact_sign_product(first_direction, second_direction);
            Classification::Decided((
                exact_sign_product(cross, factor),
                exact_sign_product(dot, factor),
            ))
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

#[allow(clippy::too_many_arguments)]
fn exact_selected_circle_retained_parallel_tangent_cross_and_dot(
    source_fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    at_start: bool,
    _parallel: &BezierParallel2,
    source_parallel: &BezierParallel2,
    source_range: &CurveParameterRange2,
    parameter: &BezierParameter2,
    selected_source_parameter: Option<
        &crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2,
    >,
    source_direction: RealSign,
    policy: &CurveContext,
) -> Classification<(RealSign, Option<RealSign>)> {
    let mut cross_uncertainty = None;
    let cross = match fragment.endpoint_tangent_cross_retained_parallel(
        at_start,
        source_parallel,
        parameter,
        source_direction,
        policy,
    ) {
        Ok(Classification::Decided(cross)) => cross,
        Ok(Classification::Uncertain(reason)) => {
            // This is an optional local tangent predicate. Independently
            // represented source carriers can make it inconclusive even when
            // the authored carriers retain an exact positive-length overlap;
            // try that stronger topology certificate below before blocking.
            cross_uncertainty = Some(reason);
            None
        }
        Err(_) => {
            cross_uncertainty = Some(UncertaintyReason::Unsupported);
            None
        }
    };
    if let Some(cross @ (RealSign::Negative | RealSign::Positive)) = cross {
        return Classification::Decided((cross, None));
    }
    if cross == Some(RealSign::Zero) {
        let zero = Real::zero();
        let one = Real::one();
        match fragment.endpoint_tangent_cross_dot_linear_combination_retained_parallel(
            at_start,
            source_parallel,
            parameter,
            source_direction,
            &zero,
            &one,
            policy,
        ) {
            Ok(Classification::Decided(Some(dot))) => {
                return Classification::Decided((RealSign::Zero, Some(dot)));
            }
            Ok(Classification::Decided(None)) => {}
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        }
    }
    match fragment.endpoint_tangent_dot_retained_parallel_source_overlap(
        source_fragment,
        at_start,
        source_parallel,
        source_range,
        parameter,
        selected_source_parameter,
        source_direction,
        policy,
    ) {
        Ok(Classification::Decided(Some(dot))) => {
            Classification::Decided((RealSign::Zero, Some(dot)))
        }
        Ok(Classification::Decided(None)) => {
            Classification::Uncertain(cross_uncertainty.unwrap_or(UncertaintyReason::Unsupported))
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

#[allow(clippy::too_many_arguments)]
fn exact_selected_circle_pair_tangent_cross_and_dot(
    first_source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    first: &crate::BezierAlgebraicCuspSemicircleFragment2,
    first_start: bool,
    second_source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second_start: bool,
    policy: &CurveContext,
) -> Classification<(RealSign, Option<RealSign>)> {
    match first.endpoint_pair_tangent_cross_and_dot(first_start, second, second_start, policy) {
        Ok(Classification::Decided(Some(relation))) => {
            return Classification::Decided(relation);
        }
        Ok(Classification::Decided(None)) => {}
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    }
    match first.endpoint_pair_tangent_cross_and_dot_source_circle(
        first_source,
        first_start,
        second,
        second_source,
        second_start,
        policy,
    ) {
        Ok(Classification::Decided(Some((cross, dot)))) => {
            Classification::Decided((cross, Some(dot)))
        }
        Ok(Classification::Decided(None)) => {
            let fallback = exact_selected_circle_pair_tangent_cross_and_dot_by_chords(
                first,
                first_start,
                second,
                second_start,
                policy,
            );
            match fallback {
                Classification::Decided(Some((cross, dot))) => {
                    Classification::Decided((cross, dot))
                }
                Classification::Decided(None) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

/// General exact tangent relation for selected-circle endpoints whose
/// construction maps do not share one specialized pair authority.
///
/// Each endpoint can already publish a certified-distinct point one unit
/// along its traversal tangent. Retaining those two witness chords lets the
/// complete chord predicates sign cross and dot without adjoining either
/// circle center field or materializing a tangent vector.
fn exact_selected_circle_pair_tangent_cross_and_dot_by_chords(
    first: &crate::BezierAlgebraicCuspSemicircleFragment2,
    first_start: bool,
    second: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second_start: bool,
    policy: &CurveContext,
) -> Classification<Option<(RealSign, Option<RealSign>)>> {
    let first = match first.endpoint_tangent_chord(first_start, policy) {
        Ok(Classification::Decided(Some(chord))) => chord,
        Ok(Classification::Decided(None)) => return Classification::Decided(None),
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let second = match second.endpoint_tangent_chord(second_start, policy) {
        Ok(Classification::Decided(Some(chord))) => chord,
        Ok(Classification::Decided(None)) => return Classification::Decided(None),
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let cross = match first.tangent_cross_sign(&second, policy) {
        Ok(Classification::Decided(cross)) => cross,
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    if cross != RealSign::Zero {
        return Classification::Decided(Some((cross, None)));
    }
    match first.tangent_dot_sign(&second, policy) {
        Ok(Classification::Decided(dot)) => Classification::Decided(Some((cross, Some(dot)))),
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

/// Signs two retained circular traversal tangents from their current endpoint
/// witnesses when no shared source-frame specialization applies.
///
/// Offset construction can give the same circular carrier either an ordinary
/// endpoint tangent or a chord-contact tangent. Replay shared pair provenance
/// first; unrelated representations still publish exact unit-step tangent
/// chords for the general predicate.
fn exact_current_selected_circle_pair_tangent_cross_and_dot(
    first: &crate::BezierAlgebraicCuspSemicircleFragment2,
    first_start: bool,
    second: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second_start: bool,
    policy: &CurveContext,
) -> Classification<(RealSign, Option<RealSign>)> {
    match first.endpoint_pair_tangent_cross_and_dot(first_start, second, second_start, policy) {
        Ok(Classification::Decided(Some(relation))) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-pair-tangent",
                "retained-pair",
            );
            return Classification::Decided(relation);
        }
        Ok(Classification::Decided(None)) => {}
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    }
    match exact_selected_circle_pair_tangent_cross_and_dot_by_chords(
        first,
        first_start,
        second,
        second_start,
        policy,
    ) {
        Classification::Decided(Some(relation)) => Classification::Decided(relation),
        Classification::Decided(None) => Classification::Uncertain(UncertaintyReason::Unsupported),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn exact_offset_tangent_relation_is_opposite(
    relation: Classification<(RealSign, Option<RealSign>)>,
) -> Classification<bool> {
    match relation {
        Classification::Decided((RealSign::Negative | RealSign::Positive, _)) => {
            Classification::Decided(false)
        }
        Classification::Decided((RealSign::Zero, Some(RealSign::Negative))) => {
            Classification::Decided(true)
        }
        Classification::Decided((RealSign::Zero, Some(RealSign::Positive))) => {
            Classification::Decided(false)
        }
        Classification::Decided((RealSign::Zero, Some(RealSign::Zero))) => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        Classification::Decided((RealSign::Zero, None)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn curve_tangent_cross_sign(
    first: &CurveTangent2,
    second: &CurveTangent2,
    policy: &CurveContext,
) -> Classification<RealSign> {
    match (first, second) {
        (
            CurveTangent2::RepresentedDirection(first),
            CurveTangent2::RepresentedDirection(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "vector-vector",
            );
            match real_sign(&offset_vector_cross(first, second), policy) {
                Some(sign) => Classification::Decided(sign),
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        (
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
            CurveTangent2::RepresentedDirection(vector),
        ) => exact_retained_parallel_tangent_cross_and_dot_vector(
            parallel,
            parameter,
            *source_direction,
            vector,
            policy,
        )
        .map(|(cross, _)| cross),
        (
            CurveTangent2::RepresentedDirection(vector),
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
        ) => exact_retained_parallel_tangent_cross_and_dot_vector(
            parallel,
            parameter,
            *source_direction,
            vector,
            policy,
        )
        .map(|(cross, _)| exact_sign_reverse(cross)),
        (
            CurveTangent2::RetainedParallel {
                parallel: first_parallel,
                parameter: first_parameter,
                source_direction: first_direction,
                ..
            },
            CurveTangent2::RetainedParallel {
                parallel: second_parallel,
                parameter: second_parameter,
                source_direction: second_direction,
                ..
            },
        ) => exact_retained_parallel_tangent_pair_cross_and_dot(
            first_parallel,
            first_parameter,
            *first_direction,
            second_parallel,
            second_parameter,
            *second_direction,
            policy,
        )
        .map(|(cross, _)| cross),
        (CurveTangent2::AlgebraicChord(first), CurveTangent2::AlgebraicChord(second)) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "algebraic-chord-algebraic-chord",
            );
            if let Some(sign) = first.tangent_cross_sign_with_shared_endpoint(second, policy) {
                return match sign {
                    Ok(sign) => sign,
                    Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
                };
            }
            match first.tangent_cross_sign(second, policy) {
                Ok(sign) => sign,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::AlgebraicChord(chord),
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
        ) => exact_algebraic_chord_retained_parallel_relation(
            chord,
            parallel,
            parameter,
            *source_direction,
            true,
            policy,
        ),
        (
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
            CurveTangent2::AlgebraicChord(chord),
        ) => exact_algebraic_chord_retained_parallel_relation(
            chord,
            parallel,
            parameter,
            *source_direction,
            true,
            policy,
        )
        .map(exact_sign_reverse),
        (CurveTangent2::AlgebraicChord(first), CurveTangent2::RepresentedDirection(second)) => {
            match first.tangent_cross_vector_sign(second, policy) {
                Ok(sign) => sign,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (CurveTangent2::RepresentedDirection(first), CurveTangent2::AlgebraicChord(second)) => {
            match second.tangent_cross_vector_sign(first, policy) {
                Ok(sign) => sign.map(exact_sign_reverse),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::CircularPoint {
                point,
                circle,
                clockwise,
            },
            CurveTangent2::RepresentedDirection(second),
        ) => exact_circular_tangent_cross_vector(point, circle, *clockwise, second, policy),
        (
            CurveTangent2::RepresentedDirection(first),
            CurveTangent2::CircularPoint {
                point,
                circle,
                clockwise,
            },
        ) => exact_circular_tangent_cross_vector(point, circle, *clockwise, first, policy)
            .map(exact_sign_reverse),
        (
            CurveTangent2::AlgebraicChord(first),
            CurveTangent2::CircularPoint {
                point,
                circle,
                clockwise,
            },
        ) => match first.certified_unit_tangent() {
            Some(tangent) => {
                exact_circular_tangent_cross_vector(point, circle, *clockwise, &tangent, policy)
                    .map(exact_sign_reverse)
            }
            None => Classification::Uncertain(UncertaintyReason::Unsupported),
        },
        (
            CurveTangent2::CircularPoint {
                point,
                circle,
                clockwise,
            },
            CurveTangent2::AlgebraicChord(second),
        ) => match second.certified_unit_tangent() {
            Some(tangent) => {
                exact_circular_tangent_cross_vector(point, circle, *clockwise, &tangent, policy)
            }
            None => Classification::Uncertain(UncertaintyReason::Unsupported),
        },
        (
            CurveTangent2::SelectedCircularEndpoint {
                fragment, at_start, ..
            },
            CurveTangent2::RepresentedDirection(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-endpoint-vector",
            );
            match fragment.endpoint_tangent_cross_vector(*at_start, second, policy) {
                Ok(cross) => cross,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::RepresentedDirection(first),
            CurveTangent2::SelectedCircularEndpoint {
                fragment, at_start, ..
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "vector-selected-circle-endpoint",
            );
            match fragment.endpoint_tangent_cross_vector(*at_start, first, policy) {
                Ok(cross) => cross.map(exact_sign_reverse),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
            CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter,
                source_direction,
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-retained-parallel",
            );
            exact_selected_circle_retained_parallel_tangent_cross_and_dot(
                source_fragment,
                fragment,
                *at_start,
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter.as_ref(),
                *source_direction,
                policy,
            )
            .map(|(cross, _)| cross)
        }
        (
            CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter,
                source_direction,
            },
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "retained-parallel-selected-circle",
            );
            exact_selected_circle_retained_parallel_tangent_cross_and_dot(
                source_fragment,
                fragment,
                *at_start,
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter.as_ref(),
                *source_direction,
                policy,
            )
            .map(|(cross, _)| exact_sign_reverse(cross))
        }
        (
            CurveTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
                ..
            },
            CurveTangent2::RepresentedDirection(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "circle-chord-contact-vector",
            );
            match exact_algebraic_chord_vector_factor(chord, second, policy) {
                Classification::Decided(factor) => {
                    Classification::Decided(exact_sign_product(*circle_cross_chord, factor))
                }
                Classification::Uncertain(_) => fragment
                    .endpoint_tangent_cross_vector(*at_start, second, policy)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            }
        }
        (
            CurveTangent2::RepresentedDirection(first),
            CurveTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
                ..
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "vector-circle-chord-contact",
            );
            match exact_algebraic_chord_vector_factor(chord, first, policy) {
                Classification::Decided(factor) => Classification::Decided(exact_sign_reverse(
                    exact_sign_product(*circle_cross_chord, factor),
                )),
                Classification::Uncertain(_) => fragment
                    .endpoint_tangent_cross_vector(*at_start, first, policy)
                    .map(|cross| cross.map(exact_sign_reverse))
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            }
        }
        (
            CurveTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
                ..
            },
            CurveTangent2::AlgebraicChord(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "circle-chord-contact-algebraic-chord",
            );
            match exact_algebraic_chord_parallel_factor(chord, second, policy) {
                Classification::Decided(factor) => {
                    Classification::Decided(exact_sign_product(*circle_cross_chord, factor))
                }
                Classification::Uncertain(_) => fragment
                    .endpoint_tangent_cross_algebraic_chord(*at_start, second, true, policy)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            }
        }
        (
            CurveTangent2::AlgebraicChord(first),
            CurveTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
                ..
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "algebraic-chord-circle-chord-contact",
            );
            match exact_algebraic_chord_parallel_factor(chord, first, policy) {
                Classification::Decided(factor) => Classification::Decided(exact_sign_reverse(
                    exact_sign_product(*circle_cross_chord, factor),
                )),
                Classification::Uncertain(_) => fragment
                    .endpoint_tangent_cross_algebraic_chord(*at_start, first, true, policy)
                    .map(|cross| cross.map(exact_sign_reverse))
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            }
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
            CurveTangent2::AlgebraicChord(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-algebraic-chord",
            );
            fragment
                .endpoint_tangent_cross_algebraic_chord_from_source(
                    source_fragment,
                    *at_start,
                    second,
                    true,
                    policy,
                )
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        (
            CurveTangent2::AlgebraicChord(first),
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "algebraic-chord-selected-circle",
            );
            fragment
                .endpoint_tangent_cross_algebraic_chord_from_source(
                    source_fragment,
                    *at_start,
                    first,
                    true,
                    policy,
                )
                .map(|cross| cross.map(exact_sign_reverse))
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment: first_source,
                fragment: first_fragment,
                at_start: first_start,
            },
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment: second_source,
                fragment: second_fragment,
                at_start: second_start,
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-pair-contact",
            );
            exact_selected_circle_pair_tangent_cross_and_dot(
                first_source,
                first_fragment,
                *first_start,
                second_source,
                second_fragment,
                *second_start,
                policy,
            )
            .map(|(cross, _)| cross)
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::ChordContact {
                fragment: second,
                at_start: second_start,
                ..
            },
        )
        | (
            CurveTangent2::ChordContact {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::SelectedCircularEndpoint {
                fragment: second,
                at_start: second_start,
                ..
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-chord-contact",
            );
            exact_current_selected_circle_pair_tangent_cross_and_dot(
                first,
                *first_start,
                second,
                *second_start,
                policy,
            )
            .map(|(cross, _)| cross)
        }
        (
            CurveTangent2::ChordContact {
                chord: first,
                circle_cross_chord: RealSign::Zero,
                ..
            },
            CurveTangent2::ChordContact {
                chord: second,
                circle_cross_chord: RealSign::Zero,
                ..
            },
        ) => exact_algebraic_chord_parallel_factor(first, second, policy).map(|_| RealSign::Zero),
        (
            CurveTangent2::ChordContact {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::ChordContact {
                fragment: second,
                at_start: second_start,
                ..
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "chord-contact-pair",
            );
            exact_current_selected_circle_pair_tangent_cross_and_dot(
                first,
                *first_start,
                second,
                *second_start,
                policy,
            )
            .map(|(cross, _)| cross)
        }
        (CurveTangent2::ChordContact { .. }, CurveTangent2::CircularPoint { .. })
        | (CurveTangent2::CircularPoint { .. }, CurveTangent2::ChordContact { .. })
        | (CurveTangent2::CircularPoint { .. }, CurveTangent2::CircularPoint { .. })
        | (CurveTangent2::SelectedCircularEndpoint { .. }, CurveTangent2::CircularPoint { .. })
        | (CurveTangent2::CircularPoint { .. }, CurveTangent2::SelectedCircularEndpoint { .. })
        | (CurveTangent2::RetainedParallel { .. }, _)
        | (_, CurveTangent2::RetainedParallel { .. }) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
    }
}

fn curve_tangents_are_opposite(
    first: &CurveTangent2,
    second: &CurveTangent2,
    policy: &CurveContext,
) -> Classification<bool> {
    match curve_tangent_cross_sign(first, second, policy) {
        Classification::Decided(RealSign::Negative | RealSign::Positive) => {
            return Classification::Decided(false);
        }
        Classification::Decided(RealSign::Zero) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    match (first, second) {
        (
            CurveTangent2::RepresentedDirection(first),
            CurveTangent2::RepresentedDirection(second),
        ) => {
            if offset_vectors_are_structurally_opposite(first, second) {
                return Classification::Decided(true);
            }
            let dot = &first.0 * &second.0 + &first.1 * &second.1;
            match real_sign(&dot, policy) {
                Some(RealSign::Negative) => Classification::Decided(true),
                Some(RealSign::Positive) => Classification::Decided(false),
                Some(RealSign::Zero) => Classification::Uncertain(UncertaintyReason::Boundary),
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        (
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
            CurveTangent2::RepresentedDirection(vector),
        )
        | (
            CurveTangent2::RepresentedDirection(vector),
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
        ) => match exact_retained_parallel_tangent_cross_and_dot_vector(
            parallel,
            parameter,
            *source_direction,
            vector,
            policy,
        ) {
            Classification::Decided((_, RealSign::Negative)) => Classification::Decided(true),
            Classification::Decided((_, RealSign::Positive)) => Classification::Decided(false),
            Classification::Decided((_, RealSign::Zero)) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (
            CurveTangent2::RetainedParallel {
                parallel: first_parallel,
                parameter: first_parameter,
                source_direction: first_direction,
                ..
            },
            CurveTangent2::RetainedParallel {
                parallel: second_parallel,
                parameter: second_parameter,
                source_direction: second_direction,
                ..
            },
        ) => match exact_retained_parallel_tangent_pair_cross_and_dot(
            first_parallel,
            first_parameter,
            *first_direction,
            second_parallel,
            second_parameter,
            *second_direction,
            policy,
        ) {
            Classification::Decided((_, RealSign::Negative)) => Classification::Decided(true),
            Classification::Decided((_, RealSign::Positive)) => Classification::Decided(false),
            Classification::Decided((_, RealSign::Zero)) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (CurveTangent2::AlgebraicChord(first), CurveTangent2::AlgebraicChord(second)) => {
            match first.tangent_dot_sign(second, policy) {
                Ok(Classification::Decided(RealSign::Negative)) => Classification::Decided(true),
                Ok(Classification::Decided(RealSign::Positive)) => Classification::Decided(false),
                Ok(Classification::Decided(RealSign::Zero)) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::AlgebraicChord(chord),
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
        )
        | (
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
            CurveTangent2::AlgebraicChord(chord),
        ) => match exact_algebraic_chord_retained_parallel_relation(
            chord,
            parallel,
            parameter,
            *source_direction,
            false,
            policy,
        ) {
            Classification::Decided(RealSign::Negative) => Classification::Decided(true),
            Classification::Decided(RealSign::Positive) => Classification::Decided(false),
            Classification::Decided(RealSign::Zero) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (CurveTangent2::AlgebraicChord(chord), CurveTangent2::RepresentedDirection(vector))
        | (CurveTangent2::RepresentedDirection(vector), CurveTangent2::AlgebraicChord(chord)) => {
            match chord.tangent_dot_vector_sign(vector, policy) {
                Ok(Classification::Decided(RealSign::Negative)) => Classification::Decided(true),
                Ok(Classification::Decided(RealSign::Positive)) => Classification::Decided(false),
                Ok(Classification::Decided(RealSign::Zero)) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment: first_source,
                fragment: first_fragment,
                at_start: first_start,
            },
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment: second_source,
                fragment: second_fragment,
                at_start: second_start,
            },
        ) => exact_offset_tangent_relation_is_opposite(
            exact_selected_circle_pair_tangent_cross_and_dot(
                first_source,
                first_fragment,
                *first_start,
                second_source,
                second_fragment,
                *second_start,
                policy,
            ),
        ),
        (
            CurveTangent2::SelectedCircularEndpoint {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::ChordContact {
                fragment: second,
                at_start: second_start,
                ..
            },
        )
        | (
            CurveTangent2::ChordContact {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::SelectedCircularEndpoint {
                fragment: second,
                at_start: second_start,
                ..
            },
        ) => exact_offset_tangent_relation_is_opposite(
            exact_current_selected_circle_pair_tangent_cross_and_dot(
                first,
                *first_start,
                second,
                *second_start,
                policy,
            ),
        ),
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
            CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter,
                source_direction,
            },
        )
        | (
            CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter,
                source_direction,
            },
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
        ) => exact_offset_tangent_relation_is_opposite(
            exact_selected_circle_retained_parallel_tangent_cross_and_dot(
                source_fragment,
                fragment,
                *at_start,
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter.as_ref(),
                *source_direction,
                policy,
            ),
        ),
        (
            CurveTangent2::SelectedCircularEndpoint {
                fragment, at_start, ..
            },
            CurveTangent2::RepresentedDirection(vector),
        )
        | (
            CurveTangent2::RepresentedDirection(vector),
            CurveTangent2::SelectedCircularEndpoint {
                fragment, at_start, ..
            },
        ) => {
            let perpendicular = (-vector.1.clone(), vector.0.clone());
            match fragment.endpoint_tangent_cross_vector(*at_start, &perpendicular, policy) {
                Ok(Classification::Decided(RealSign::Negative)) => Classification::Decided(true),
                Ok(Classification::Decided(RealSign::Positive)) => Classification::Decided(false),
                Ok(Classification::Decided(RealSign::Zero)) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
            CurveTangent2::AlgebraicChord(chord),
        )
        | (
            CurveTangent2::AlgebraicChord(chord),
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
        ) => {
            match fragment.endpoint_tangent_dot_algebraic_chord_from_source(
                source_fragment,
                *at_start,
                chord,
                policy,
            ) {
                Ok(Classification::Decided(RealSign::Negative)) => Classification::Decided(true),
                Ok(Classification::Decided(RealSign::Positive)) => Classification::Decided(false),
                Ok(Classification::Decided(RealSign::Zero)) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::ChordContact {
                chord,
                circle_dot_chord: Some(circle_dot_chord),
                ..
            },
            CurveTangent2::AlgebraicChord(candidate),
        )
        | (
            CurveTangent2::AlgebraicChord(candidate),
            CurveTangent2::ChordContact {
                chord,
                circle_dot_chord: Some(circle_dot_chord),
                ..
            },
        ) => match exact_algebraic_chord_parallel_factor(chord, candidate, policy) {
            Classification::Decided(factor) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-tangent-dot",
                    "selected-chord-normal-algebraic-chord",
                );
                match exact_sign_product(*circle_dot_chord, factor) {
                    RealSign::Negative => Classification::Decided(true),
                    RealSign::Positive => Classification::Decided(false),
                    RealSign::Zero => Classification::Uncertain(UncertaintyReason::Boundary),
                }
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (
            CurveTangent2::ChordContact {
                chord,
                circle_dot_chord: Some(circle_dot_chord),
                ..
            },
            CurveTangent2::RepresentedDirection(candidate),
        )
        | (
            CurveTangent2::RepresentedDirection(candidate),
            CurveTangent2::ChordContact {
                chord,
                circle_dot_chord: Some(circle_dot_chord),
                ..
            },
        ) => match exact_algebraic_chord_vector_factor(chord, candidate, policy) {
            Classification::Decided(factor) => {
                match exact_sign_product(*circle_dot_chord, factor) {
                    RealSign::Negative => Classification::Decided(true),
                    RealSign::Positive => Classification::Decided(false),
                    RealSign::Zero => Classification::Uncertain(UncertaintyReason::Boundary),
                }
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (
            CurveTangent2::ChordContact {
                chord: first_chord,
                circle_dot_chord: Some(first_dot),
                ..
            },
            CurveTangent2::ChordContact {
                chord: second_chord,
                circle_dot_chord: Some(second_dot),
                ..
            },
        ) => match exact_algebraic_chord_parallel_factor(first_chord, second_chord, policy) {
            Classification::Decided(factor) => {
                match exact_sign_product(exact_sign_product(*first_dot, *second_dot), factor) {
                    RealSign::Negative => Classification::Decided(true),
                    RealSign::Positive => Classification::Decided(false),
                    RealSign::Zero => Classification::Uncertain(UncertaintyReason::Boundary),
                }
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (
            CurveTangent2::ChordContact {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::ChordContact {
                fragment: second,
                at_start: second_start,
                ..
            },
        ) => exact_offset_tangent_relation_is_opposite(
            exact_current_selected_circle_pair_tangent_cross_and_dot(
                first,
                *first_start,
                second,
                *second_start,
                policy,
            ),
        ),
        (CurveTangent2::CircularPoint { .. }, _) | (_, CurveTangent2::CircularPoint { .. }) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        (CurveTangent2::ChordContact { .. }, CurveTangent2::RepresentedDirection(_))
        | (CurveTangent2::RepresentedDirection(_), CurveTangent2::ChordContact { .. })
        | (CurveTangent2::ChordContact { .. }, CurveTangent2::AlgebraicChord(_))
        | (CurveTangent2::AlgebraicChord(_), CurveTangent2::ChordContact { .. })
        | (CurveTangent2::RetainedParallel { .. }, _)
        | (_, CurveTangent2::RetainedParallel { .. }) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
    }
}

fn offset_vectors_are_structurally_opposite(first: &(Real, Real), second: &(Real, Real)) -> bool {
    (&first.0 + &second.0).zero_status() == hyperreal::ZeroKnowledge::Zero
        && (&first.1 + &second.1).zero_status() == hyperreal::ZeroKnowledge::Zero
}

struct RetainedDeferredArcContact2 {
    source_parameter: CurveParameter2,
    source_at_start: bool,
    source_at_end: bool,
    point: CurvePoint2,
    fillet_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    fillet_half: u8,
}

struct RetainedDeferredArcFilletResult2 {
    fillet_fragments: Vec<BezierSplitFragment2>,
    arc_replacement: Option<Vec<BezierSplitFragment2>>,
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
            data.certified_loop_fill_rules = self.data.certified_loop_fill_rules.clone();
            data.regularized_filled_left_policy = self.data.regularized_filled_left_policy;
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
        let fill_rules = material_contours
            .iter()
            .chain(&hole_contours)
            .map(Contour2::fill_rule)
            .collect::<Arc<[_]>>();
        let mut data = CurveRegionData2::new(boundary_loops);
        data.certified_loop_roles = Some(roles);
        data.certified_loop_fill_rules = Some(fill_rules);
        // The caller's arrangement already certified these oriented, merged
        // line contours. Preserve that proof when choosing the compact native
        // representation, so the next operation does not normalize again.
        data.regularized_filled_left_policy = Some(policy.retained_object_policy());
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
            data.certified_loop_fill_rules = Some(Arc::from(fill_rules));
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
            data.certified_loop_fill_rules = self.data.certified_loop_fill_rules.clone();
            data.regularized_filled_left_policy =
                retained_regularized_topology.then(|| policy.retained_object_policy());
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
            data.certified_loop_fill_rules = Some(Arc::from(fill_rules));
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
        let retained = policy
            .retained_object_policy_with_dependencies(self.data.regularized_filled_left_policy);
        let data = self.data_mut_for_construction();
        data.filled_side_is_left
            .certify(Arc::from(vec![true; loop_count]));
        if loop_count == 1 && data.certified_loop_roles.is_none() {
            data.certified_loop_roles = Some(shared_all_material_curve_region_loop_roles(1));
        }
        data.regularized_filled_left_policy = Some(retained);
        Ok(self)
    }

    pub(crate) fn has_regularized_filled_left_topology(&self, policy: &CurveContext) -> bool {
        let Some(retained) = self.data.regularized_filled_left_policy else {
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

    /// Returns authoritative per-loop fill rules when retained by construction.
    ///
    /// Region promotion preserves the source contour rules. Curved regions
    /// built only from boundary paths currently return `None`, meaning their
    /// simple-loop topology uses the kernel's default parity behavior.
    pub fn loop_fill_rules(&self) -> Option<&[FillRule]> {
        self.data.certified_loop_fill_rules.as_deref()
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
            let retained_policy = attempt.retained_object_policy_with_dependencies(
                normalized.data.regularized_filled_left_policy,
            );
            Ok(profiles
                .into_iter()
                .map(|profile| {
                    let indices = std::iter::once(profile.material_loop_index)
                        .chain(profile.hole_loop_indices.iter().copied());
                    let fill_rules = normalized.loop_fill_rules().map(|rules| {
                        indices
                            .clone()
                            .map(|index| rules[index])
                            .collect::<Arc<[_]>>()
                    });
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
                    data.certified_loop_fill_rules = fill_rules;
                    // Removing other complete material components preserves
                    // this component's regularized boundary. Ownership and
                    // the source normalization remain decision dependencies.
                    data.regularized_filled_left_policy = Some(retained_policy);
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
        let fill_rules = self.loop_fill_rules().map_or_else(
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
                .or(self.data.regularized_filled_left_policy)
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
            .loop_fill_rules()
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
            data.certified_loop_fill_rules = Some(Arc::from(fill_rules));
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
            data.certified_loop_fill_rules = None;
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
                .certified_loop_fill_rules
                .as_deref()
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
            .certified_loop_fill_rules
            .as_ref()
            .is_some_and(|rules| rules.len() != contours.len())
        {
            return Err(CurveError::Topology(
                "curve-region fill-rule count is inconsistent with line contours".into(),
            ));
        }

        let mut material = Vec::new();
        let mut holes = Vec::new();
        for (index, (contour, role)) in contours.iter().zip(roles).enumerate() {
            let contour = match &self.data.certified_loop_fill_rules {
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
                .certified_loop_fill_rules
                .as_ref()
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
                .certified_loop_fill_rules
                .as_ref()
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
                        self.data.certified_loop_fill_rules.as_deref(),
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
                self.data.certified_loop_fill_rules.as_deref(),
            );
        };
        if self
            .data
            .certified_loop_roles
            .as_ref()
            .is_some_and(|roles| roles.len() != native_loops.len())
            || self
                .data
                .certified_loop_fill_rules
                .as_ref()
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
                .certified_loop_fill_rules
                .as_ref()
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
                .certified_loop_fill_rules
                .as_ref()
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
                .certified_loop_fill_rules
                .as_ref()
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
            .certified_loop_fill_rules
            .as_ref()
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
                .certified_loop_fill_rules
                .as_ref()
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
    let denominator_sign = multiply_algebraic_ray_signs(point.denominator_sign(), weight_sign);
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
            Classification::Decided(sign) => multiply_algebraic_ray_signs(sign, denominator_sign),
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
                    multiply_algebraic_ray_signs(derivative_sign, denominator_sign)
                        == RealSign::Positive;
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

const fn multiply_algebraic_ray_signs(first: RealSign, second: RealSign) -> RealSign {
    match (first, second) {
        (RealSign::Zero, _) | (_, RealSign::Zero) => RealSign::Zero,
        (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
            RealSign::Positive
        }
        (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive) => {
            RealSign::Negative
        }
    }
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
