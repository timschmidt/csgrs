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

use std::sync::Arc;
use std::sync::OnceLock;

use hyperreal::{Real, RealSign};
use hypersolve::AlgebraicRootRepresentation;
use hypersolve::{
    AlgebraicFiberRootCountStatus, BivariatePolynomial, CurveResultantParameter,
    count_bivariate_common_fiber_roots_at_algebraic_parameter,
};

use crate::BezierParameterPolynomial;
use crate::RationalBezierAlgebraicPointImage2;
use crate::bezier::BezierParallelLineTangentContact2;
use crate::bezier_algebraic_image::RationalBezierAlgebraicPointPredicate2;
use crate::bezier_arrangement::represented_roots_equal;
use crate::bezier_moment::RationalQuadraticAreaIntegralCache;
use crate::bezier_offset::BezierAlgebraicCuspSemicircleSimilarityCache2;
use crate::bezier_offset::{
    BezierAlgebraicChordAxisDirection2, BezierAlgebraicFiberProjection2,
    algebraic_chord_point_linear_order_to_exact, algebraic_selected_correlated_predicate_sign,
    algebraic_selected_fiber_parameters, bivariate_fiber_strict_sign_on_parameter_range,
    retained_point_linear_difference_to_algebraic_sign,
};
use crate::bezier_split::BezierSelectedFiberSource2;
use crate::bezier_topology::exact_polynomial_line_contact_relation_from_direction;
use crate::classify::LineSide;
use crate::classify::{compare_reals, is_zero, real_sign};
use crate::curve::RetainedFilletRadialFrame2;
use crate::curve::{
    CornerTrimCut2, RetainedFilletFrame2, exact_corner_carrier, solve_exact_chamfer_corner,
    solve_exact_fillet_corner, try_map_corner_solutions, validate_corner_design_value,
};
use crate::policy::{
    PolicyClassificationCache, PolicyEvaluationCache, resolve_cached_classification,
    resolve_cached_evaluation, resolve_certified_operation, resolve_certified_value,
};
use crate::region::LineArcRegion2;
use crate::region_nesting::RegionArrangement2;
use crate::{
    Aabb2, Axis2, BezierAlgebraicEndpointImage2, BezierAreaMoments2, BezierArrangementGraph2,
    BezierArrangementTraversal2, BezierEndpoint, BezierEndpointPointImage2,
    BezierFlatteningOptions, BezierLineContact, BezierLineContactKind, BezierLineContactRelation,
    BezierLineCrossingDirection, BezierLineImageFitRelation, BezierParallel2,
    BezierParallelSource2, BezierParallelVerificationOptions, BezierParameter2,
    BezierParameterRange2, BezierRetainedLinearOverlapTraversal2,
    BezierRetainedRationalOverlapTraversal2, BezierSplitFragment2, BezierSubcurve2, BooleanOp,
    CircularArc2, Classification, Contour2, ContourPointLocation, CubicBezier2, Curve2,
    CurveCertainty, CurveContext, CurveCornerMode2, CurveCornerSolutions2, CurveError,
    CurveFamily2, CurveGeometry2, CurveIntersectionPairBlockerKind2, CurveOperation2, CurveOutcome,
    CurvePath2, CurvePathIntersectionContact2, CurveRegionParameter2, CurveRegionParameterRange2,
    CurveResult, ExactCurveError, ExactCurveResult, FillRule, LineSeg2, OffsetCornerStyle2, Point2,
    QuadraticBezier2, RationalBezier2, RationalBezierIntersectionPointEvidence2,
    RationalBezierPointIncidence2, RationalQuadraticBezier2, RegionPointLocation,
    RetainedTopologyStatus, Segment2, SegmentKindCounts, UncertaintyReason,
};

/// A closed native Bezier/conic boundary loop.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBoundaryLoop2 {
    fragments: Vec<BezierSubcurve2>,
}

/// A closed retained Bezier/conic boundary loop.
///
/// This carrier may contain retained analytic and algebraic fragments,
/// including endpoint images, chords, and cusp joins, in addition to native
/// [`BezierBoundaryLoop2`] fragments.
/// It is a concrete exact-object region boundary in the exactness model's sense: the algebraic
/// pieces remain replayable construction evidence, not sampled coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionBoundaryLoop2 {
    fragments: Vec<BezierSplitFragment2>,
    arrangement_sources: Option<Vec<CurveRegionFragmentSource2>>,
}

/// Arrangement provenance for one retained boundary fragment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CurveRegionFragmentSource2 {
    arrangement_fragment_index: usize,
    source_curve_index: usize,
    source_fragment_index: usize,
}

impl CurveRegionFragmentSource2 {
    /// Constructs retained fragment provenance from arrangement graph indices.
    pub const fn new(
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

    /// Returns the source curve index carried by the graph fragment.
    pub const fn source_curve_index(self) -> usize {
        self.source_curve_index
    }

    /// Returns the split-fragment index within the source curve materialization.
    pub const fn source_fragment_index(self) -> usize {
        self.source_fragment_index
    }
}

/// A higher-order retained region built from accepted native/algebraic carriers.
///
/// This is the first region object for decided retained traversals containing
/// algebraic endpoint-image fragments. It intentionally does not flatten or
/// approximate those fragments and it does not claim a finite area integral for
/// them. Construction and decision remain separate; native polynomial subloops
/// reuse the Green-integral path described above.
#[derive(Clone)]
pub struct CurveRegion2 {
    data: Arc<CurveRegionData2>,
}

struct CurveRegionData2 {
    boundary_loops: Vec<CurveRegionBoundaryLoop2>,
    certified_loop_roles: Option<Arc<[CurveRegionLoopRole]>>,
    certified_loop_fill_rules: Option<Arc<[FillRule]>>,
    signed_loop_composition: bool,
    certified_regularized_filled_left_topology: bool,
    strict_materialized_connectivity_certified: bool,
    filled_side_is_left: PolicyClassificationCache<Arc<[bool]>>,
    native_boundary_loops: OnceLock<Option<Arc<[BezierBoundaryLoop2]>>>,
    native_boundary_bounds: PolicyClassificationCache<Arc<[Aabb2]>>,
    line_image_region: PolicyClassificationCache<Option<LineArcRegion2>>,
    retained_rational_evaluators: OnceLock<CurveResult<Vec<Vec<Option<RationalBezier2>>>>>,
    signed_area_cache: PolicyEvaluationCache<Option<Real>>,
    axis_aligned_algebraic_source_loops: OnceLock<(
        CurveContext,
        CurveCertainty,
        Arc<[Option<Arc<ExactAxisAlignedAlgebraicSourceLoop2>>]>,
    )>,
}

impl CurveRegionData2 {
    fn new(boundary_loops: Vec<CurveRegionBoundaryLoop2>) -> Self {
        let strict_materialized_connectivity_certified =
            retained_region_has_strict_materialized_connectivity(&boundary_loops);
        Self {
            boundary_loops,
            certified_loop_roles: None,
            certified_loop_fill_rules: None,
            signed_loop_composition: false,
            certified_regularized_filled_left_topology: false,
            strict_materialized_connectivity_certified,
            filled_side_is_left: PolicyClassificationCache::new(),
            native_boundary_loops: OnceLock::new(),
            native_boundary_bounds: PolicyClassificationCache::new(),
            line_image_region: PolicyClassificationCache::new(),
            retained_rational_evaluators: OnceLock::new(),
            signed_area_cache: PolicyEvaluationCache::new(),
            axis_aligned_algebraic_source_loops: OnceLock::new(),
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

/// Furthest exact stage reached by unified native-boundary arrangement.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveRegionArrangementStage2 {
    /// The unordered endpoint graph was being assembled into closed rings.
    RingAssembly,
    /// Checked contours were being assigned material/hole roles.
    RegionRoleAssignment,
}

/// Immediate native line/arc arrangement with a unified curved output.
#[derive(Clone, Debug)]
pub struct CurveRegionArrangement2 {
    region: Option<CurveRegion2>,
    fill_rule: FillRule,
    source_segment_count: usize,
    stage: CurveRegionArrangementStage2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
    output_ring_count: Option<usize>,
    output_boundary_segment_count: Option<usize>,
    output_boundary_segment_kind_counts: Option<SegmentKindCounts>,
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

/// Evidence for a general-curve offset routed through exact-scalar certified segmentation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionSegmentedOffsetEvidence2 {
    used_exact_authoritative_path: bool,
    max_source_chord_error: Real,
    loop_evidence: Vec<CurveRegionSegmentationLoopEvidence2>,
    lossy_boundary: bool,
}

/// A unified offset region with retained certified source-segmentation evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionSegmentedOffsetResult2 {
    region: CurveRegion2,
    evidence: CurveRegionSegmentedOffsetEvidence2,
}

/// Per-loop evidence for a certified polynomial/rational parallel boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionCertifiedParallelLoopEvidence2 {
    role: CurveRegionLoopRole,
    fill_rule: FillRule,
    signed_left_distance: Real,
    source_curve_count: usize,
    output_curve_count: usize,
    exact_source_curve_count: usize,
    approximated_source_curve_count: usize,
    verification_leaf_count: usize,
}

/// Evidence for the strongest completed lane of a general curved-region offset.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionCertifiedParallelOffsetEvidence2 {
    used_exact_authoritative_path: bool,
    used_certified_parallel_path: bool,
    used_segmented_source_fallback: bool,
    max_parallel_fit_error: Real,
    max_output_chord_error: Real,
    certified_pre_regularization_boundary_error: Option<Real>,
    final_boundary_hausdorff_certified: bool,
    loop_evidence: Vec<CurveRegionCertifiedParallelLoopEvidence2>,
    fallback_evidence: Option<CurveRegionSegmentedOffsetEvidence2>,
}

/// Unified region produced by exact native, certified parallel, or explicit fallback offsetting.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionCertifiedParallelOffsetResult2 {
    region: CurveRegion2,
    evidence: CurveRegionCertifiedParallelOffsetEvidence2,
}

impl CurveRegionArrangement2 {
    /// Returns the materialized unified region, if role assignment succeeded.
    pub const fn region(&self) -> Option<&CurveRegion2> {
        self.region.as_ref()
    }

    /// Returns the materialized output or the retained arrangement blocker.
    pub fn region_classification(&self) -> Classification<&CurveRegion2> {
        match self.region() {
            Some(region) => Classification::Decided(region),
            None => {
                Classification::Uncertain(self.blocker().unwrap_or(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Returns the fill rule used by the completed native arrangement.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns the number of exact source segments evaluated by the arrangement.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the final exact stage reached by the arrangement.
    pub const fn stage(&self) -> CurveRegionArrangementStage2 {
        self.stage
    }

    /// Returns the final retained topology status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the blocker when the completed arrangement could not materialize a region.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }

    /// Returns output ring count when role assignment completed.
    pub const fn output_ring_count(&self) -> Option<usize> {
        self.output_ring_count
    }

    /// Returns output boundary segment count when role assignment completed.
    pub const fn output_boundary_segment_count(&self) -> Option<usize> {
        self.output_boundary_segment_count
    }

    /// Returns output native primitive-family counts when role assignment completed.
    pub const fn output_boundary_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_boundary_segment_kind_counts
    }

    /// Consumes the result and returns its unified region, if materialized.
    pub fn into_region(self) -> Option<CurveRegion2> {
        self.region
    }
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

impl CurveRegionSegmentedOffsetEvidence2 {
    /// Returns true when the authoritative exact offset completed without segmentation.
    pub const fn used_exact_authoritative_path(&self) -> bool {
        self.used_exact_authoritative_path
    }

    /// Returns the certified curve-to-source-chord error budget.
    pub const fn max_source_chord_error(&self) -> &Real {
        &self.max_source_chord_error
    }

    /// Returns source-loop segmentation evidence.
    pub fn loop_evidence(&self) -> &[CurveRegionSegmentationLoopEvidence2] {
        &self.loop_evidence
    }

    /// Returns whether the operation crossed an explicitly lossy segmentation boundary.
    pub const fn lossy_boundary(&self) -> bool {
        self.lossy_boundary
    }
}

impl CurveRegionSegmentedOffsetResult2 {
    /// Returns the unified offset region.
    pub const fn region(&self) -> &CurveRegion2 {
        &self.region
    }

    /// Returns the exact-fast-path or certified-segmentation evidence.
    pub const fn evidence(&self) -> &CurveRegionSegmentedOffsetEvidence2 {
        &self.evidence
    }

    /// Consumes the result and returns the unified offset region.
    pub fn into_region(self) -> CurveRegion2 {
        self.region
    }

    /// Consumes the result into geometry and evidence.
    pub fn into_parts(self) -> (CurveRegion2, CurveRegionSegmentedOffsetEvidence2) {
        (self.region, self.evidence)
    }
}

impl CurveRegionCertifiedParallelLoopEvidence2 {
    /// Returns the retained material/hole role.
    pub const fn role(&self) -> CurveRegionLoopRole {
        self.role
    }

    /// Returns the authored fill rule.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns the signed distance applied along traversal-left normals.
    pub const fn signed_left_distance(&self) -> &Real {
        &self.signed_left_distance
    }

    /// Returns the number of authored source curves.
    pub const fn source_curve_count(&self) -> usize {
        self.source_curve_count
    }

    /// Returns the number of output exact or fitted curves.
    pub const fn output_curve_count(&self) -> usize {
        self.output_curve_count
    }

    /// Returns how many source curves retained an exact parallel carrier.
    pub const fn exact_source_curve_count(&self) -> usize {
        self.exact_source_curve_count
    }

    /// Returns how many source curves required certified polynomial fitting.
    pub const fn approximated_source_curve_count(&self) -> usize {
        self.approximated_source_curve_count
    }

    /// Returns the aggregate conservative-verifier leaf count.
    pub const fn verification_leaf_count(&self) -> usize {
        self.verification_leaf_count
    }
}

impl CurveRegionCertifiedParallelOffsetEvidence2 {
    /// Returns whether the authoritative exact offset kernel completed the operation.
    pub const fn used_exact_authoritative_path(&self) -> bool {
        self.used_exact_authoritative_path
    }

    /// Returns whether verified polynomial/rational parallels supplied the output boundary.
    pub const fn used_certified_parallel_path(&self) -> bool {
        self.used_certified_parallel_path
    }

    /// Returns whether completion required the legacy source-chord fallback.
    pub const fn used_segmented_source_fallback(&self) -> bool {
        self.used_segmented_source_fallback
    }

    /// Returns the requested per-span bound to each exact analytic parallel.
    pub const fn max_parallel_fit_error(&self) -> &Real {
        &self.max_parallel_fit_error
    }

    /// Returns the requested chord bound used to regularize the produced path.
    pub const fn max_output_chord_error(&self) -> &Real {
        &self.max_output_chord_error
    }

    /// Returns the directed analytic-parallel-to-emitted-chord bound before regularization.
    ///
    /// `None` identifies the weaker source-chord fallback. Arrangement
    /// regularization may remove raw offset branches, so this value is not
    /// promoted to a Hausdorff certificate for the final region boundary.
    pub const fn certified_pre_regularization_boundary_error(&self) -> Option<&Real> {
        self.certified_pre_regularization_boundary_error.as_ref()
    }

    /// Returns whether the final regularized boundary itself has a Hausdorff certificate.
    pub const fn final_boundary_hausdorff_certified(&self) -> bool {
        self.final_boundary_hausdorff_certified
    }

    /// Returns per-loop exact/fitted construction evidence.
    pub fn loop_evidence(&self) -> &[CurveRegionCertifiedParallelLoopEvidence2] {
        &self.loop_evidence
    }

    /// Returns the legacy fallback evidence when that lane was required.
    pub const fn fallback_evidence(&self) -> Option<&CurveRegionSegmentedOffsetEvidence2> {
        self.fallback_evidence.as_ref()
    }
}

impl CurveRegionCertifiedParallelOffsetResult2 {
    /// Returns the regularized unified offset region.
    pub const fn region(&self) -> &CurveRegion2 {
        &self.region
    }

    /// Returns construction and certification evidence.
    pub const fn evidence(&self) -> &CurveRegionCertifiedParallelOffsetEvidence2 {
        &self.evidence
    }

    /// Consumes the result and returns its region.
    pub fn into_region(self) -> CurveRegion2 {
        self.region
    }

    /// Consumes the result into geometry and evidence.
    pub fn into_parts(self) -> (CurveRegion2, CurveRegionCertifiedParallelOffsetEvidence2) {
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
                "signed_loop_composition",
                &self.data.signed_loop_composition,
            )
            .field(
                "certified_regularized_filled_left_topology",
                &self.data.certified_regularized_filled_left_topology,
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
                && self.data.signed_loop_composition == other.data.signed_loop_composition
                && self.data.certified_regularized_filled_left_topology
                    == other.data.certified_regularized_filled_left_topology)
    }
}

/// Filled side of an oriented closed curve boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveBoundaryInteriorSide2 {
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

/// Exact role assignment for retained line-image Bezier boundary loops.
///
/// This evidence is intentionally narrower than arbitrary retained Bezier role
/// assignment.  It accepts materialized Bezier/conic fragments only through a
/// certified exact line-image fit, accepts algebraic endpoint-image fragments
/// only when they provide exact endpoint witnesses, lowers those loops to
/// native [`Contour2`] line loops, and then runs exact nesting.  This follows
/// the exact-geometric-computation boundary: unsupported curve families
/// remain explicit evidence gaps rather than being sampled into polygon
/// surrogates.  The source counters retain whether role assignment consumed
/// native fit certificates or algebraic endpoint evidence.  The containment
/// step uses boundary-first point-in-contour classification as surveyed by
/// boundary-first winding classification.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionLineRoleEvidence2 {
    roles: Vec<CurveRegionLoopRole>,
    nesting_depths: Vec<usize>,
    materialized_fragment_count: usize,
    algebraic_fragment_count: usize,
    contours: Vec<Contour2>,
    loop_arrangement_sources: Option<Vec<Option<Vec<CurveRegionFragmentSource2>>>>,
}

/// Exact orientation-derived role assignment for native retained Bezier loops.
///
/// This evidence is broader than [`CurveRegionLineRoleEvidence2`]: it
/// accepts native polynomial Bezier and rational quadratic conic loops whenever
/// their exact Green-integral signed area is implemented and nonzero.  It is
/// intentionally narrower than full curved-loop nesting: it assigns roles from
/// the authored loop orientation only, returns the signed areas as evidence,
/// and rejects algebraic, unresolved, zero-area, or unsupported-area loops.
/// That keeps the construction/decision boundary explicit in the exactness model's sense; see
/// exact-computation discipline.  The signed-area evidence comes from Green's theorem
/// and Bernstein/rational Bezier identities as described by the Bernstein and de Casteljau curve model.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionSignedAreaRoleEvidence2 {
    roles: Vec<CurveRegionLoopRole>,
    signed_areas: Vec<Real>,
    loop_fragment_counts: Option<Vec<usize>>,
    loop_arrangement_sources: Option<Vec<Option<Vec<CurveRegionFragmentSource2>>>>,
}

/// Exact nesting-derived role assignment for native retained curved loops.
///
/// Unlike [`CurveRegionLineRoleEvidence2`], this evidence does not lower
/// nonlinear loops to line contours. Unlike
/// [`CurveRegionSignedAreaRoleEvidence2`], it does not trust authored
/// orientation to distinguish material from holes. It chooses an exact
/// representative point on each candidate loop and classifies it against every
/// other native Bezier/conic loop by counting certified ray crossings. Boundary
/// hits, tangent-only ray contacts, algebraic carriers, unresolved line-contact
/// predicates, and unsupported area/zero-area loops remain explicit
/// uncertainty. The crossing rule is the exact-object analogue of the
/// point-in-polygon method surveyed by boundary-first winding classification
/// 131-144; all branch decisions follow exact-computation discipline.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionNestingRoleEvidence2 {
    roles: Vec<CurveRegionLoopRole>,
    nesting_depths: Vec<usize>,
    signed_areas: Vec<Real>,
    sample_points: Vec<Point2>,
    loop_fragment_counts: Option<Vec<usize>>,
    loop_arrangement_sources: Option<Vec<Option<Vec<CurveRegionFragmentSource2>>>>,
}

impl CurveRegionLineRoleEvidence2 {
    /// Constructs a retained line-image role evidence.
    pub fn new(
        roles: Vec<CurveRegionLoopRole>,
        nesting_depths: Vec<usize>,
        materialized_fragment_count: usize,
        algebraic_fragment_count: usize,
        contours: Vec<Contour2>,
    ) -> CurveResult<Self> {
        validate_evidence_length(roles.len(), "nesting depth", nesting_depths.len())?;
        validate_evidence_length(roles.len(), "line contour", contours.len())?;
        validate_nesting_depth_roles(&roles, &nesting_depths)?;
        validate_line_role_evidence_fragment_counts(
            materialized_fragment_count,
            algebraic_fragment_count,
            &contours,
        )?;
        Ok(Self {
            roles,
            nesting_depths,
            materialized_fragment_count,
            algebraic_fragment_count,
            contours,
            loop_arrangement_sources: None,
        })
    }

    /// Attaches one optional arrangement source trail per retained loop.
    pub fn with_loop_arrangement_sources(
        mut self,
        loop_arrangement_sources: Vec<Option<Vec<CurveRegionFragmentSource2>>>,
    ) -> CurveResult<Self> {
        validate_loop_arrangement_sources(self.roles.len(), &loop_arrangement_sources)?;
        validate_line_loop_arrangement_source_counts(&self.contours, &loop_arrangement_sources)?;
        self.loop_arrangement_sources = Some(loop_arrangement_sources);
        Ok(self)
    }

    /// Returns one assigned role per retained boundary loop.
    pub fn roles(&self) -> &[CurveRegionLoopRole] {
        &self.roles
    }

    /// Returns the certified count of containing loops for each retained loop.
    pub fn nesting_depths(&self) -> &[usize] {
        &self.nesting_depths
    }

    /// Returns how many materialized fragments contributed certified line-image fits.
    pub const fn materialized_fragment_count(&self) -> usize {
        self.materialized_fragment_count
    }

    /// Returns how many algebraic endpoint-image fragments contributed exact endpoints.
    pub const fn algebraic_fragment_count(&self) -> usize {
        self.algebraic_fragment_count
    }

    /// Returns true when algebraic endpoint evidence contributed to the line contours.
    pub const fn has_algebraic_fragments(&self) -> bool {
        self.algebraic_fragment_count > 0
    }

    /// Returns exact native line contours used for role assignment.
    pub fn contours(&self) -> &[Contour2] {
        &self.contours
    }

    /// Returns per-loop arrangement/source provenance when the evidence has it.
    pub fn loop_arrangement_sources(&self) -> Option<&[Option<Vec<CurveRegionFragmentSource2>>]> {
        self.loop_arrangement_sources.as_deref()
    }

    /// Returns loop indices assigned as material.
    pub fn material_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| (*role == CurveRegionLoopRole::Material).then_some(index))
            .collect()
    }

    /// Returns loop indices assigned as holes.
    pub fn hole_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| (*role == CurveRegionLoopRole::Hole).then_some(index))
            .collect()
    }

    /// Builds the unified owned region represented by this exact role evidence.
    pub fn try_to_curve_region(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveRegion2>> {
        let mut material = Vec::new();
        let mut holes = Vec::new();
        for (contour, role) in self
            .contours
            .iter()
            .cloned()
            .zip(self.roles.iter().copied())
        {
            match role {
                CurveRegionLoopRole::Material => material.push(contour),
                CurveRegionLoopRole::Hole => holes.push(contour),
            }
        }
        CurveRegion2::try_from_native_contours(material, holes, policy)
    }
}

impl CurveRegionSignedAreaRoleEvidence2 {
    /// Constructs a retained signed-area role evidence.
    pub fn new(
        roles: Vec<CurveRegionLoopRole>,
        signed_areas: Vec<Real>,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        validate_evidence_length(roles.len(), "signed area", signed_areas.len())?;
        validate_signed_area_roles(&roles, &signed_areas, policy)?;
        Ok(Self {
            roles,
            signed_areas,
            loop_fragment_counts: None,
            loop_arrangement_sources: None,
        })
    }

    fn with_loop_fragment_counts(mut self, loop_fragment_counts: Vec<usize>) -> CurveResult<Self> {
        validate_loop_fragment_counts(self.roles.len(), &loop_fragment_counts)?;
        self.loop_fragment_counts = Some(loop_fragment_counts);
        Ok(self)
    }

    /// Attaches one optional arrangement source trail per retained loop.
    pub fn with_loop_arrangement_sources(
        mut self,
        loop_arrangement_sources: Vec<Option<Vec<CurveRegionFragmentSource2>>>,
    ) -> CurveResult<Self> {
        validate_loop_arrangement_sources(self.roles.len(), &loop_arrangement_sources)?;
        validate_counted_loop_arrangement_source_counts(
            self.loop_fragment_counts.as_deref(),
            &loop_arrangement_sources,
        )?;
        self.loop_arrangement_sources = Some(loop_arrangement_sources);
        Ok(self)
    }

    /// Returns one assigned role per retained boundary loop.
    pub fn roles(&self) -> &[CurveRegionLoopRole] {
        &self.roles
    }

    /// Returns exact signed areas used as orientation evidence.
    pub fn signed_areas(&self) -> &[Real] {
        &self.signed_areas
    }

    /// Returns per-loop arrangement/source provenance when the evidence has it.
    pub fn loop_arrangement_sources(&self) -> Option<&[Option<Vec<CurveRegionFragmentSource2>>]> {
        self.loop_arrangement_sources.as_deref()
    }

    /// Returns loop indices assigned as material.
    pub fn material_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| (*role == CurveRegionLoopRole::Material).then_some(index))
            .collect()
    }

    /// Returns loop indices assigned as holes.
    pub fn hole_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| (*role == CurveRegionLoopRole::Hole).then_some(index))
            .collect()
    }
}

impl CurveRegionNestingRoleEvidence2 {
    /// Constructs a retained curved-loop nesting role evidence.
    pub fn new(
        roles: Vec<CurveRegionLoopRole>,
        nesting_depths: Vec<usize>,
        signed_areas: Vec<Real>,
        sample_points: Vec<Point2>,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        validate_evidence_length(roles.len(), "nesting depth", nesting_depths.len())?;
        validate_evidence_length(roles.len(), "signed area", signed_areas.len())?;
        validate_evidence_length(roles.len(), "sample point", sample_points.len())?;
        validate_nesting_depth_roles(&roles, &nesting_depths)?;
        validate_nonzero_signed_area_evidence(&signed_areas, policy)?;
        Ok(Self {
            roles,
            nesting_depths,
            signed_areas,
            sample_points,
            loop_fragment_counts: None,
            loop_arrangement_sources: None,
        })
    }

    fn with_loop_fragment_counts(mut self, loop_fragment_counts: Vec<usize>) -> CurveResult<Self> {
        validate_loop_fragment_counts(self.roles.len(), &loop_fragment_counts)?;
        self.loop_fragment_counts = Some(loop_fragment_counts);
        Ok(self)
    }

    /// Attaches one optional arrangement source trail per retained loop.
    pub fn with_loop_arrangement_sources(
        mut self,
        loop_arrangement_sources: Vec<Option<Vec<CurveRegionFragmentSource2>>>,
    ) -> CurveResult<Self> {
        validate_loop_arrangement_sources(self.roles.len(), &loop_arrangement_sources)?;
        validate_counted_loop_arrangement_source_counts(
            self.loop_fragment_counts.as_deref(),
            &loop_arrangement_sources,
        )?;
        self.loop_arrangement_sources = Some(loop_arrangement_sources);
        Ok(self)
    }

    /// Returns one assigned role per retained boundary loop.
    pub fn roles(&self) -> &[CurveRegionLoopRole] {
        &self.roles
    }

    /// Returns the certified count of containing loops for each retained loop.
    pub fn nesting_depths(&self) -> &[usize] {
        &self.nesting_depths
    }

    /// Returns exact signed areas used to certify nondegenerate native loops.
    pub fn signed_areas(&self) -> &[Real] {
        &self.signed_areas
    }

    /// Returns exact sample points used for nesting classification.
    pub fn sample_points(&self) -> &[Point2] {
        &self.sample_points
    }

    /// Returns per-loop arrangement/source provenance when the evidence has it.
    pub fn loop_arrangement_sources(&self) -> Option<&[Option<Vec<CurveRegionFragmentSource2>>]> {
        self.loop_arrangement_sources.as_deref()
    }

    /// Returns loop indices assigned as material.
    pub fn material_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| (*role == CurveRegionLoopRole::Material).then_some(index))
            .collect()
    }

    /// Returns loop indices assigned as holes.
    pub fn hole_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| (*role == CurveRegionLoopRole::Hole).then_some(index))
            .collect()
    }
}

impl BezierBoundaryLoop2 {
    /// Constructs a closed boundary loop from native Bezier/conic fragments.
    pub fn new(fragments: Vec<BezierSubcurve2>, policy: &CurveContext) -> CurveResult<Self> {
        validate_native_boundary_loop(&fragments, policy)?;
        Ok(Self { fragments })
    }

    pub(crate) fn from_policy_validated_fragments(fragments: Vec<BezierSubcurve2>) -> Self {
        debug_assert!(!fragments.is_empty());
        Self { fragments }
    }

    /// Returns native curve fragments in loop order.
    pub fn fragments(&self) -> &[BezierSubcurve2] {
        &self.fragments
    }

    /// Consumes the loop and returns native curve fragments.
    pub fn into_fragments(self) -> Vec<BezierSubcurve2> {
        self.fragments
    }

    /// Returns the number of native fragments in the loop.
    pub fn len(&self) -> usize {
        self.fragments.len()
    }

    /// Returns true when the loop contains no fragments.
    pub fn is_empty(&self) -> bool {
        self.fragments.is_empty()
    }

    /// Returns the exact signed area for loops with implemented area integrals.
    ///
    /// Polynomial Beziers use exact polynomial Green integrals. Rational
    /// quadratics use the homogeneous rational Green integral when their
    /// denominator is certified nonzero on the affine parameter interval.
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

    /// Returns exact signed area and first moments when every retained boundary
    /// fragment has an implemented symbolic integral.
    ///
    /// Polynomial Béziers, polynomial-equivalent rational Béziers, finite
    /// rational quadratics, their exact homogeneous degree elevations,
    /// arbitrary-degree rational carriers with at-most-quadratic weight
    /// polynomials, certified cubic-weight carriers, and arbitrary-degree
    /// weight carriers that rational-root deflation reduces to linear factors
    /// plus either an exact power of one irreducible quadratic or a quartic
    /// product of two are integrated directly. `None` preserves another
    /// genuinely rational boundary whose first-moment integral is not yet
    /// implemented; it never requests a flattening tolerance.
    pub fn area_moments(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Option<BezierAreaMoments2>>>> {
        resolve_certified_operation(policy, |attempt| self.area_moments_raw(attempt))
    }

    pub(crate) fn area_moments_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAreaMoments2>>> {
        if self.fragments.is_empty() {
            return Err(CurveError::Topology(
                "Bezier boundary loop moments require nonempty fragments".to_owned(),
            ));
        }
        let mut total = BezierAreaMoments2::zero();
        for fragment in &self.fragments {
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

    /// Classifies an exact point against this curved boundary loop.
    ///
    /// The classifier uses exact point incidence followed by a certified
    /// horizontal-ray crossing count. It does not flatten curved fragments.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<ContourPointLocation>> {
        classify_point_against_native_loop(self, point, policy)
    }
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
        }
    }
}

fn validate_native_boundary_loop(
    fragments: &[BezierSubcurve2],
    policy: &CurveContext,
) -> CurveResult<()> {
    if fragments.is_empty() {
        return Err(CurveError::Topology(
            "Bezier boundary loop requires nonempty fragments".to_owned(),
        ));
    }

    for (left, right) in fragments
        .iter()
        .zip(fragments.iter().cycle().skip(1))
        .take(fragments.len())
    {
        let (_, left_end) = left.endpoint_refs();
        let (right_start, _) = right.endpoint_refs();
        if !certified_points_equal(left_end, right_start, policy) {
            return Err(CurveError::Topology(
                "Bezier boundary loop fragments must be endpoint-connected and closed".to_owned(),
            ));
        }
    }
    Ok(())
}

fn certified_points_equal(left: &Point2, right: &Point2, policy: &CurveContext) -> bool {
    left == right
        || (is_zero(&(left.x() - right.x()), policy) == Some(true)
            && is_zero(&(left.y() - right.y()), policy) == Some(true))
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
}

fn validate_bezier_region_loops<Loop>(boundary_loops: &[Loop]) -> CurveResult<()>
where
    Loop: PartialEq,
{
    for (index, boundary_loop) in boundary_loops.iter().enumerate() {
        if boundary_loops[index + 1..].contains(boundary_loop) {
            return Err(CurveError::Topology(
                "Bezier region must not duplicate boundary loop evidence".to_owned(),
            ));
        }
    }
    Ok(())
}

impl CurveRegionBoundaryLoop2 {
    /// Constructs a retained boundary loop from accepted split fragments.
    pub fn new(fragments: Vec<BezierSplitFragment2>, policy: &CurveContext) -> CurveResult<Self> {
        validate_retained_boundary_loop(&fragments, policy)?;
        Ok(Self {
            fragments,
            arrangement_sources: None,
        })
    }

    /// Constructs a retained boundary loop with one source record per fragment.
    pub fn try_new_with_arrangement_sources(
        fragments: Vec<BezierSplitFragment2>,
        arrangement_sources: Vec<CurveRegionFragmentSource2>,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        validate_retained_boundary_loop(&fragments, policy)?;
        if fragments.len() != arrangement_sources.len() {
            return Err(CurveError::Topology(
                "retained boundary source count does not match fragment count".to_owned(),
            ));
        }
        validate_retained_boundary_loop_sources(&arrangement_sources)?;
        Ok(Self {
            fragments,
            arrangement_sources: Some(arrangement_sources),
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
        ))
    }

    fn from_certified_arrangement_chain(
        fragments: Vec<BezierSplitFragment2>,
        arrangement_sources: Vec<CurveRegionFragmentSource2>,
    ) -> Self {
        debug_assert!(!fragments.is_empty());
        debug_assert_eq!(fragments.len(), arrangement_sources.len());
        Self {
            fragments,
            arrangement_sources: Some(arrangement_sources),
        }
    }

    /// Returns retained split fragments in loop order.
    pub fn fragments(&self) -> &[BezierSplitFragment2] {
        &self.fragments
    }

    /// Consumes the loop and returns retained split fragments.
    pub fn into_fragments(self) -> Vec<BezierSplitFragment2> {
        self.fragments
    }

    pub(crate) fn without_arrangement_sources(mut self) -> Self {
        self.arrangement_sources = None;
        self
    }

    /// Returns arrangement/source indices for graph-built loops, when retained.
    pub fn arrangement_sources(&self) -> Option<&[CurveRegionFragmentSource2]> {
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
        self.fragments.iter().any(|fragment| {
            matches!(
                fragment,
                BezierSplitFragment2::AlgebraicEndpointImages { .. }
                    | BezierSplitFragment2::AnalyticParallel(_)
                    | BezierSplitFragment2::AlgebraicChord(_)
                    | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            )
        })
    }

    /// Returns exact signed area only for fully native loops with implemented integrals.
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
            if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
                let Some(line) = chord.exact_line() else {
                    return Ok(Classification::Decided(None));
                };
                total = &total
                    + &crate::contour::line_signed_area_contribution(line.start(), line.end())?;
                continue;
            }
            let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
                return Ok(Classification::Decided(None));
            };
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
            };
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
            if !start.is_exact() || !end.is_exact() {
                return Err(CurveError::Topology(
                    "retained materialized Bezier fragment must carry exact range boundaries"
                        .into(),
                ));
            }
            validate_retained_fragment_parameter_order(start, end, policy)
        }
        BezierSplitFragment2::AlgebraicEndpointImages {
            start,
            end,
            source_curve,
            start_image,
            end_image,
            ..
        } => {
            if source_curve.is_some() {
                validate_retained_fragment_parameter_order(start, end, policy)?;
            }
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
            if chord.policy() != *policy {
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
        BezierSplitFragment2::Unresolved { .. } => Err(CurveError::Topology(
            "retained Bezier region boundary loops must not contain unresolved carriers".into(),
        )),
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
    source_curve: &Option<BezierSubcurve2>,
    image: Option<&crate::BezierAlgebraicEndpointImage2>,
    policy: &CurveContext,
) -> CurveResult<()> {
    match boundary {
        BezierParameter2::Exact(_) => {
            if image.is_some() {
                return Err(CurveError::Topology(
                    "retained exact endpoint must not carry algebraic endpoint image evidence"
                        .into(),
                ));
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
            if !image.is_transformed() && !(source_curve.is_some() && image.is_lazy_first_order()) {
                return Err(CurveError::Topology(
                    "retained algebraic endpoint image must be transformed or retain replayable first-order source evidence".into(),
                ));
            }
            if let Some(source_curve) = source_curve {
                let expected = crate::BezierAlgebraicEndpointImage2::from_source_curve(
                    source_curve,
                    parameter,
                    policy,
                )?;
                if !image.matches_required_source_evidence(&expected) {
                    return Err(CurveError::Topology(
                        "retained algebraic endpoint image does not match retained source curve"
                            .into(),
                    ));
                }
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
    validate_bezier_region_loops(boundary_loops)?;
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

#[derive(Clone, Debug, PartialEq)]
struct RetainedEndpointEvidence {
    point: Option<Point2>,
    retained_point: Option<crate::RationalBezierIntersectionPointEvidence2>,
    algebraic: Option<(
        Box<AlgebraicRootRepresentation>,
        Box<AlgebraicRootRepresentation>,
    )>,
    source: Option<(BezierSubcurve2, BezierParameter2)>,
    analytic_source: Option<(BezierParallel2, BezierParameter2)>,
    algebraic_cusp_source: Option<(crate::BezierAlgebraicCuspSemicircleFragment2, bool)>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RetainedEndpointEquality {
    Equal,
    NotEqual,
    Uncertified,
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
        let left_end = retained_fragment_endpoint_evidence(left, false, policy)?;
        let right_start = retained_fragment_endpoint_evidence(right, true, policy)?;
        match retained_endpoint_equality(&left_end, &right_start, policy) {
            RetainedEndpointEquality::Equal => {}
            RetainedEndpointEquality::NotEqual => {
                return Err(CurveError::Topology(
                    "retained Bezier boundary loop fragments must be endpoint-connected and closed"
                        .into(),
                ));
            }
            RetainedEndpointEquality::Uncertified => {
                return Err(CurveError::Topology(
                    "retained Bezier boundary loop must carry certified endpoint connectivity evidence"
                        .into(),
                ));
            }
        }
    }
    Ok(())
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

        let left_end = retained_fragment_endpoint_evidence(left.fragment(), false, policy)?;
        let right_start = retained_fragment_endpoint_evidence(right.fragment(), true, policy)?;
        match retained_endpoint_equality(&left_end, &right_start, policy) {
            RetainedEndpointEquality::Equal => {}
            RetainedEndpointEquality::NotEqual => {
                return Err(CurveError::Topology(
                    "retained arrangement chain contains disconnected fragments".into(),
                ));
            }
            RetainedEndpointEquality::Uncertified => {
                return Err(CurveError::Topology(
                    "retained arrangement chain endpoint connectivity is uncertified".into(),
                ));
            }
        }
    }
    Ok(())
}

fn retained_fragment_endpoint_evidence(
    fragment: &BezierSplitFragment2,
    start_endpoint: bool,
    policy: &CurveContext,
) -> CurveResult<RetainedEndpointEvidence> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            let (start, end) = curve.endpoints();
            Ok(RetainedEndpointEvidence {
                point: Some(if start_endpoint {
                    start.clone()
                } else {
                    end.clone()
                }),
                retained_point: Some(crate::RationalBezierIntersectionPointEvidence2::Exact(
                    if start_endpoint { start } else { end },
                )),
                algebraic: None,
                source: None,
                analytic_source: None,
                algebraic_cusp_source: None,
            })
        }
        BezierSplitFragment2::AlgebraicEndpointImages {
            reversed,
            start,
            end,
            source_curve,
            start_image,
            end_image,
        } => {
            let source_start_endpoint = start_endpoint != *reversed;
            let parameter = if source_start_endpoint { start } else { end };
            let image = if source_start_endpoint {
                start_image.as_ref()
            } else {
                end_image.as_ref()
            };
            let source = source_curve
                .as_ref()
                .map(|source_curve| (source_curve.clone(), parameter.clone()));
            let point = retained_endpoint_point_evidence(parameter, image, source_curve, policy)?;
            let algebraic = image.and_then(retained_endpoint_algebraic_evidence);
            let retained_point = match source_curve {
                Some(source_curve) => {
                    let rational = RationalBezier2::try_from_subcurve(source_curve)?;
                    crate::rational_bezier_general::exact_contact_point_evidence(
                        &rational, parameter, policy,
                    )?
                }
                None => None,
            };
            let analytic_source = source_curve
                .as_ref()
                .map(|source_curve| {
                    match source_curve {
                        BezierSubcurve2::Quadratic(source) => source.parallel_left(Real::zero()),
                        BezierSubcurve2::Cubic(source) => source.parallel_left(Real::zero()),
                        BezierSubcurve2::RationalQuadratic(source) => {
                            source.parallel_left(Real::zero())
                        }
                        BezierSubcurve2::Rational(source) => source.parallel_left(Real::zero()),
                    }
                    .map(|parallel| (parallel, parameter.clone()))
                })
                .transpose()?;
            Ok(RetainedEndpointEvidence {
                point,
                retained_point,
                algebraic,
                source,
                analytic_source,
                algebraic_cusp_source: None,
            })
        }
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            let parameter = if start_endpoint != fragment.is_reversed() {
                fragment.range().start()
            } else {
                fragment.range().end()
            };
            let point = match parameter.as_exact() {
                Some(parameter) => match fragment.parallel().point_at(parameter, policy)? {
                    Classification::Decided(point) => Some(point),
                    Classification::Uncertain(_) => None,
                },
                None => None,
            };
            let retained_point = point.is_none().then(|| {
                crate::RationalBezierIntersectionPointEvidence2::AnalyticParallel(
                    crate::BezierAnalyticParallelPoint2::new(
                        fragment.parallel().clone(),
                        parameter.clone(),
                        policy,
                    ),
                )
            });
            Ok(RetainedEndpointEvidence {
                point,
                retained_point,
                algebraic: None,
                source: None,
                analytic_source: Some((fragment.parallel().clone(), parameter.clone())),
                algebraic_cusp_source: None,
            })
        }
        BezierSplitFragment2::AlgebraicChord(chord) => {
            let endpoint = if start_endpoint {
                chord.start()
            } else {
                chord.end()
            };
            let (point, algebraic) = match endpoint {
                crate::RationalBezierIntersectionPointEvidence2::Exact(point) => {
                    (Some(point.clone()), None)
                }
                crate::RationalBezierIntersectionPointEvidence2::Algebraic(image) => (
                    image.exact_rational_point(policy),
                    image.resolved(policy).and_then(|image| {
                        Some((
                            Box::new(image.x()?.representation()?.clone()),
                            Box::new(image.y()?.representation()?.clone()),
                        ))
                    }),
                ),
                crate::RationalBezierIntersectionPointEvidence2::AlgebraicChordPair(_)
                | crate::RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_)
                | crate::RationalBezierIntersectionPointEvidence2::AlgebraicCuspChordDerived(_)
                | crate::RationalBezierIntersectionPointEvidence2::AlgebraicChordParallel(_)
                | crate::RationalBezierIntersectionPointEvidence2::AnalyticParallel(_)
                | crate::RationalBezierIntersectionPointEvidence2::Similarity(_) => (None, None),
            };
            Ok(RetainedEndpointEvidence {
                point,
                retained_point: Some(endpoint.clone()),
                algebraic,
                source: None,
                analytic_source: None,
                algebraic_cusp_source: None,
            })
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            let retained_point = match fragment.endpoint_point_evidence(start_endpoint, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(_) => None,
            };
            Ok(RetainedEndpointEvidence {
                point: fragment.endpoint_exact_point(start_endpoint, policy)?,
                retained_point,
                algebraic: None,
                source: None,
                analytic_source: fragment.endpoint_analytic_source(start_endpoint),
                algebraic_cusp_source: Some((fragment.clone(), start_endpoint)),
            })
        }
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let retained_point = if start_endpoint {
                fragment.start_point().clone()
            } else {
                fragment.end_point().clone()
            };
            let point = match &retained_point {
                crate::RationalBezierIntersectionPointEvidence2::Exact(point) => {
                    Some(point.clone())
                }
                _ => None,
            };
            Ok(RetainedEndpointEvidence {
                point,
                retained_point: Some(retained_point),
                algebraic: None,
                source: None,
                analytic_source: None,
                algebraic_cusp_source: None,
            })
        }
        BezierSplitFragment2::Unresolved { .. } => Err(CurveError::Topology(
            "retained Bezier region boundary loops must not contain unresolved carriers".into(),
        )),
    }
}

fn retained_endpoint_algebraic_evidence(
    image: &crate::BezierAlgebraicEndpointImage2,
) -> Option<(
    Box<AlgebraicRootRepresentation>,
    Box<AlgebraicRootRepresentation>,
)> {
    let (x, y) = match image.try_point().ok()? {
        BezierEndpointPointImage2::Polynomial(point) => (
            point.x()?.representation()?.clone(),
            point.y()?.representation()?.clone(),
        ),
        BezierEndpointPointImage2::Rational(point) => (
            point.x()?.representation()?.clone(),
            point.y()?.representation()?.clone(),
        ),
    };
    Some((Box::new(x), Box::new(y)))
}

fn retained_endpoint_point_evidence(
    parameter: &BezierParameter2,
    image: Option<&crate::BezierAlgebraicEndpointImage2>,
    source_curve: &Option<BezierSubcurve2>,
    policy: &CurveContext,
) -> CurveResult<Option<Point2>> {
    if let Some(image) = image
        && let Some(point) = exact_rational_point_from_image(image.point(), None)
    {
        return Ok(Some(point));
    }

    let BezierParameter2::Exact(value) = parameter else {
        return Ok(None);
    };
    let Some(source_curve) = source_curve else {
        return Ok(None);
    };
    match subcurve_point_at(source_curve, value.clone(), policy) {
        Classification::Decided(point) => Ok(Some(point)),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "could not certify retained boundary exact endpoint from source curve: {reason:?}"
        ))),
    }
}

fn retained_endpoint_equality(
    left: &RetainedEndpointEvidence,
    right: &RetainedEndpointEvidence,
    policy: &CurveContext,
) -> RetainedEndpointEquality {
    if let (Some(left), Some(right)) = (&left.source, &right.source)
        && left == right
    {
        return RetainedEndpointEquality::Equal;
    }

    if let (Some((left_parallel, left_parameter)), Some((right_parallel, right_parameter))) =
        (&left.analytic_source, &right.analytic_source)
        && left_parameter == right_parameter
        && left_parallel.shares_parameterized_curve_evidence(right_parallel)
    {
        return RetainedEndpointEquality::Equal;
    }

    if let (Some((left, left_start)), Some((right, right_start))) =
        (&left.algebraic_cusp_source, &right.algebraic_cusp_source)
        && left.shares_endpoint_evidence(*left_start, right, *right_start)
    {
        return RetainedEndpointEquality::Equal;
    }

    {
        let cusp_chord_matches =
            |cusp: &Option<(crate::BezierAlgebraicCuspSemicircleFragment2, bool)>,
             point: &Option<crate::RationalBezierIntersectionPointEvidence2>| {
                let (
                    Some((cusp, start_endpoint)),
                    Some(crate::RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(
                        point,
                    )),
                ) = (cusp, point)
                else {
                    return false;
                };
                cusp.shares_endpoint_point_evidence(*start_endpoint, point)
            };
        if cusp_chord_matches(&left.algebraic_cusp_source, &right.retained_point)
            || cusp_chord_matches(&right.algebraic_cusp_source, &left.retained_point)
        {
            return RetainedEndpointEquality::Equal;
        }
    }

    if let (Some(left), Some(right)) = (&left.point, &right.point) {
        return match is_zero(&left.distance_squared(right), policy) {
            Some(true) => RetainedEndpointEquality::Equal,
            Some(false) => RetainedEndpointEquality::NotEqual,
            None => RetainedEndpointEquality::Uncertified,
        };
    }

    if let (Some((left_x, left_y)), Some((right_x, right_y))) = (&left.algebraic, &right.algebraic)
    {
        let x_equal = represented_roots_equal(left_x, right_x, policy);
        let y_equal = represented_roots_equal(left_y, right_y, policy);
        return match (x_equal, y_equal) {
            (Some(true), Some(true)) => RetainedEndpointEquality::Equal,
            (Some(false), _) | (_, Some(false)) => RetainedEndpointEquality::NotEqual,
            _ => RetainedEndpointEquality::Uncertified,
        };
    }

    if let (Some(left), Some(right)) = (&left.retained_point, &right.retained_point) {
        match left.same_point(right, policy) {
            Classification::Decided(true) => return RetainedEndpointEquality::Equal,
            Classification::Decided(false) => return RetainedEndpointEquality::NotEqual,
            Classification::Uncertain(_) => {}
        }
    }

    RetainedEndpointEquality::Uncertified
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
                CurveGeometry2::Line(line) => Some(Segment2::Line(line.clone())),
                CurveGeometry2::CircularArc(arc) => Some(Segment2::Arc(arc.clone())),
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

fn promote_native_region_arrangement(
    arrangement: RegionArrangement2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegionArrangement2> {
    let RegionArrangement2 {
        region,
        fill_rule,
        source_segment_count,
        stage,
        status,
        blocker,
        output_ring_count,
        output_boundary_segment_count,
        output_boundary_segment_kind_counts,
    } = arrangement;
    let region = region
        .as_ref()
        .map(|region| CurveRegion2::try_from_line_arc_region_raw(region, policy))
        .transpose()?;
    Ok(CurveRegionArrangement2 {
        region,
        fill_rule,
        source_segment_count,
        stage,
        status,
        blocker,
        output_ring_count,
        output_boundary_segment_count,
        output_boundary_segment_kind_counts,
    })
}

fn curve_region_edit_error(operation: CurveOperation2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(operation, CurveFamily2::Line, cause)
}

fn retained_corner_fragment_trim(
    fragment: &BezierSplitFragment2,
    parameter: CurveRegionParameter2,
    cut_point: &RationalBezierIntersectionPointEvidence2,
    replacement_rational_curve: Option<&RationalBezier2>,
    keep_before_cut: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<BezierSplitFragment2> {
    if let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment {
        let parameter = parameter.as_algebraic_cusp().cloned().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            )
        })?;
        let keep_lower_parameter_range = keep_before_cut != fragment.is_reversed();
        let (start, end) = if keep_lower_parameter_range {
            (fragment.start_parameter().clone(), parameter)
        } else {
            (parameter, fragment.end_parameter().clone())
        };
        return match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
            fragment.semicircle().clone(),
            start,
            end,
            fragment.is_reversed(),
            policy,
        )
        .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(fragment) => {
                Ok(BezierSplitFragment2::AlgebraicCuspSemicircle(fragment))
            }
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                reason,
            )),
        };
    }
    if let BezierSplitFragment2::AnalyticParallel(fragment) = fragment {
        let parameter = parameter.as_bezier_parameter().cloned().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            )
        })?;
        let keep_lower_parameter_range = keep_before_cut != fragment.is_reversed();
        let range = if keep_lower_parameter_range {
            BezierParameterRange2::new_validated(fragment.range().start().clone(), parameter)
        } else {
            BezierParameterRange2::new_validated(parameter, fragment.range().end().clone())
        };
        return Ok(BezierSplitFragment2::AnalyticParallel(
            crate::BezierParallelFragment2::from_certified_range(
                fragment.parallel().clone(),
                range,
                fragment.is_reversed(),
            ),
        ));
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
        return crate::BezierAlgebraicChord2::from_ordered_parameter_range(
            chord, start, end, policy,
        )
        .map(BezierSplitFragment2::AlgebraicChord)
        .map_err(|cause| curve_region_edit_error(operation, cause));
    }
    let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
        return Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        ));
    };
    let replacement_curve = replacement_rational_curve
        .cloned()
        .map(BezierSubcurve2::Rational);
    let curve = replacement_curve.as_ref().unwrap_or(curve);
    if let BezierSubcurve2::Quadratic(line_curve) = curve
        && matches!(
            parameter.as_bezier_parameter(),
            Some(BezierParameter2::Algebraic(_))
        )
        && line_curve.retained_exact_line_image().is_some()
        && line_curve
            .retained_parallel_line_tangent_contacts()
            .is_empty()
    {
        let start = if keep_before_cut {
            RationalBezierIntersectionPointEvidence2::Exact(line_curve.start().clone())
        } else {
            cut_point.clone()
        };
        let end = if keep_before_cut {
            cut_point.clone()
        } else {
            RationalBezierIntersectionPointEvidence2::Exact(line_curve.end().clone())
        };
        return match crate::BezierAlgebraicChord2::try_new(start, end, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(chord) => Ok(BezierSplitFragment2::AlgebraicChord(chord)),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                reason,
            )),
        };
    }
    let parameter = parameter.as_bezier_parameter().cloned().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        )
    })?;
    let split = match curve
        .split_at_parameters_refined(&[parameter], policy)
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
        return Ok(selected);
    };
    Ok(
        match curve.retained_quadratic_representative(&CurveContext::STRICT) {
            Ok(Classification::Decided(Some(curve))) => BezierSplitFragment2::Materialized {
                start,
                end,
                curve: BezierSubcurve2::RationalQuadratic(curve),
            },
            Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => {
                BezierSplitFragment2::Materialized {
                    start,
                    end,
                    curve: BezierSubcurve2::Rational(curve),
                }
            }
        },
    )
}

fn canonicalize_retained_quadratics_in_corner_path(path: CurvePath2) -> CurvePath2 {
    if !path
        .curves()
        .iter()
        .any(|curve| matches!(curve.geometry(), CurveGeometry2::RationalBezier(_)))
    {
        return path;
    }
    let mut changed = false;
    let curves = path
        .curves()
        .iter()
        .map(|curve| {
            let CurveGeometry2::RationalBezier(rational) = curve.geometry() else {
                return curve.clone();
            };
            match rational.retained_quadratic_representative(&CurveContext::STRICT) {
                Ok(Classification::Decided(Some(quadratic))) => {
                    changed = true;
                    Curve2::from(quadratic)
                }
                Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => {
                    curve.clone()
                }
            }
        })
        .collect();
    if changed {
        CurvePath2::from_structurally_closed_curves(curves)
    } else {
        path
    }
}

fn wrap_segmented_parallel_fallback(
    segmented: Classification<CurveRegionSegmentedOffsetResult2>,
    max_parallel_fit_error: &Real,
    max_output_chord_error: &Real,
) -> ExactCurveResult<Classification<CurveRegionCertifiedParallelOffsetResult2>> {
    match segmented {
        Classification::Decided(segmented) => {
            let (region, fallback_evidence) = segmented.into_parts();
            Ok(Classification::Decided(
                CurveRegionCertifiedParallelOffsetResult2 {
                    region,
                    evidence: CurveRegionCertifiedParallelOffsetEvidence2 {
                        used_exact_authoritative_path: fallback_evidence
                            .used_exact_authoritative_path(),
                        used_certified_parallel_path: false,
                        used_segmented_source_fallback: !fallback_evidence
                            .used_exact_authoritative_path(),
                        max_parallel_fit_error: max_parallel_fit_error.clone(),
                        max_output_chord_error: max_output_chord_error.clone(),
                        certified_pre_regularization_boundary_error: fallback_evidence
                            .used_exact_authoritative_path()
                            .then(Real::zero),
                        final_boundary_hausdorff_certified: fallback_evidence
                            .used_exact_authoritative_path(),
                        loop_evidence: Vec::new(),
                        fallback_evidence: Some(fallback_evidence),
                    },
                },
            ))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn push_native_offset_component(
    role: CurveRegionLoopRole,
    component: CurveRegion2,
    material_components: &mut Vec<CurveRegion2>,
    void_components: &mut Vec<CurveRegion2>,
) {
    match role {
        CurveRegionLoopRole::Material => material_components.push(component),
        CurveRegionLoopRole::Hole => void_components.push(component),
    }
}

fn curve_region_from_native_material_contour(
    contour: Contour2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegion2> {
    CurveRegion2::try_from_native_contours_raw(vec![contour], Vec::new(), policy)
}

fn curve_region_from_optional_native_material_contour(
    contour: Option<Contour2>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegion2> {
    contour.map_or_else(
        || Ok(CurveRegion2::empty()),
        |contour| curve_region_from_native_material_contour(contour, policy),
    )
}

fn regularize_native_contour_with_curve_region(
    contour: &Contour2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegion2> {
    let path = curve_path_from_native_contour(contour)?;
    let raw = CurveRegion2::try_from_boundary_paths_with_loop_semantics_raw(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[contour.fill_rule()],
        policy,
        None,
    )?;
    raw.regularized_region_raw(policy)
}

fn regularize_native_cycles_with_curve_region(
    cycles: Vec<Contour2>,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegion2> {
    if cycles.is_empty() {
        return Ok(CurveRegion2::empty());
    }
    if fill_rule == FillRule::EvenOdd {
        let mut result = CurveRegion2::empty();
        for cycle in cycles {
            let component = curve_region_from_native_material_contour(cycle, policy)?;
            result = result
                .boolean_region_raw(&component, BooleanOp::Xor, policy)
                .map_err(|error| error.with_operation(CurveOperation2::Offset))?;
        }
        return Ok(result);
    }

    let mut paths = Vec::with_capacity(cycles.len());
    let mut roles = Vec::with_capacity(cycles.len());
    for cycle in cycles {
        let Some(area) = cycle
            .signed_area()
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        else {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Offset,
                CurveFamily2::Line,
                UncertaintyReason::Unsupported,
            ));
        };
        let role = match real_sign(&area, policy) {
            Some(RealSign::Positive) => CurveRegionLoopRole::Material,
            Some(RealSign::Negative) => CurveRegionLoopRole::Hole,
            Some(RealSign::Zero) => continue,
            None => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Offset,
                    CurveFamily2::Line,
                    UncertaintyReason::RealSign,
                ));
            }
        };
        paths.push(curve_path_from_native_contour(&cycle)?);
        roles.push(role);
    }
    if paths.is_empty() {
        return Ok(CurveRegion2::empty());
    }
    let fill_rules = vec![FillRule::NonZero; paths.len()];
    let mut raw = CurveRegion2::try_from_boundary_paths_with_loop_semantics_raw(
        &paths,
        &roles,
        &fill_rules,
        policy,
        None,
    )?;
    raw.data_mut_for_construction().signed_loop_composition = true;
    raw.regularized_region_raw(policy)
}

fn regularize_native_offset_regions(
    mut material_components: Vec<CurveRegion2>,
    void_components: Vec<CurveRegion2>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegion2> {
    if material_components.len() == 1 && void_components.is_empty() {
        return Ok(material_components
            .pop()
            .expect("single offset component inventory"));
    }
    let mut material = CurveRegion2::empty();
    for component in material_components {
        material = material
            .boolean_region_raw(&component, BooleanOp::Union, policy)
            .map_err(|error| error.with_operation(CurveOperation2::Offset))?;
    }

    if material.is_empty() || void_components.is_empty() {
        return Ok(material);
    }

    let mut voids = CurveRegion2::empty();
    for component in void_components {
        voids = voids
            .boolean_region_raw(&component, BooleanOp::Union, policy)
            .map_err(|error| error.with_operation(CurveOperation2::Offset))?;
    }
    material
        .boolean_region_raw(&voids, BooleanOp::Difference, policy)
        .map_err(|error| error.with_operation(CurveOperation2::Offset))
}

fn native_region_role_contour(
    region: &LineArcRegion2,
    role: CurveRegionLoopRole,
    ordinal: usize,
) -> Option<&Contour2> {
    match role {
        CurveRegionLoopRole::Material => region.material_contours().get(ordinal),
        CurveRegionLoopRole::Hole => region.hole_contours().get(ordinal),
    }
}

struct ExactOffsetSpan2 {
    fragments: Vec<BezierSplitFragment2>,
    source_end: RationalBezierIntersectionPointEvidence2,
    offset_start: RationalBezierIntersectionPointEvidence2,
    offset_end: RationalBezierIntersectionPointEvidence2,
    start_tangent: Option<ExactOffsetTangent2>,
    end_tangent: Option<ExactOffsetTangent2>,
}

enum ExactOffsetTangent2 {
    Vector((Real, Real)),
    AlgebraicChord(crate::BezierAlgebraicChord2),
    CircularPoint {
        point: RationalBezierIntersectionPointEvidence2,
        circle: Arc<crate::rational_bezier::RationalQuadraticCircle2>,
        clockwise: bool,
    },
    SelectedCircularEndpoint {
        fragment: crate::BezierAlgebraicCuspSemicircleFragment2,
        at_start: bool,
    },
    ChordContact {
        fragment: crate::BezierAlgebraicCuspSemicircleFragment2,
        at_start: bool,
        chord: crate::BezierAlgebraicChord2,
        circle_cross_chord: RealSign,
    },
}

fn exact_offset_tangent_is_selected_circle(tangent: &ExactOffsetTangent2) -> bool {
    matches!(
        tangent,
        ExactOffsetTangent2::SelectedCircularEndpoint { .. }
            | ExactOffsetTangent2::ChordContact { .. }
    )
}

struct ExactAxisAlignedAlgebraicOffsetSpan2 {
    source: crate::BezierAlgebraicChord2,
    offset_start: crate::RationalBezierIntersectionPointEvidence2,
    offset_end: crate::RationalBezierIntersectionPointEvidence2,
    direction: BezierAlgebraicChordAxisDirection2,
    normal_offset: (Real, Real),
}

#[derive(Clone)]
struct ExactAxisAlignedAlgebraicFiber2 {
    chord: crate::BezierAlgebraicChord2,
    direction: BezierAlgebraicChordAxisDirection2,
}

struct ExactAxisAlignedAlgebraicSourceLoop2 {
    spans: Arc<[ExactAxisAlignedAlgebraicFiber2]>,
    erosion_source: OnceLock<(
        CurveContext,
        CurveCertainty,
        Arc<ExactAxisAlignedAlgebraicErosionSource2>,
    )>,
}

struct ExactAxisAlignedAlgebraicErosionSource2 {
    source_x_fibers: Arc<[ExactAxisAlignedAlgebraicFiber2]>,
    source_y_fibers: Arc<[ExactAxisAlignedAlgebraicFiber2]>,
    source_bounds: Aabb2,
}

fn retained_chord_or_exact_line_fragment(
    chord: crate::BezierAlgebraicChord2,
) -> CurveResult<BezierSplitFragment2> {
    let (Some(start), Some(end)) = (chord.start().as_exact(), chord.end().as_exact()) else {
        return Ok(BezierSplitFragment2::AlgebraicChord(chord));
    };
    Ok(materialized_offset_fragment(BezierSubcurve2::Quadratic(
        QuadraticBezier2::from_line_segment(LineSeg2::try_new(start.clone(), end.clone())?),
    )))
}

fn exact_axis_aligned_algebraic_offset_span(
    chord: &crate::BezierAlgebraicChord2,
    direction: BezierAlgebraicChordAxisDirection2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactAxisAlignedAlgebraicOffsetSpan2>> {
    let normal_offset = direction.signed_left_offset(distance);
    let offset_start = match crate::BezierAlgebraicChord2::translated_endpoint(
        chord.start(),
        &normal_offset.0,
        &normal_offset.1,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let offset_end = match crate::BezierAlgebraicChord2::translated_endpoint(
        chord.end(),
        &normal_offset.0,
        &normal_offset.1,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(
        ExactAxisAlignedAlgebraicOffsetSpan2 {
            source: chord.clone(),
            offset_start,
            offset_end,
            direction,
            normal_offset,
        },
    ))
}

fn append_exact_algebraic_line_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    from: &crate::RationalBezierIntersectionPointEvidence2,
    to: &crate::RationalBezierIntersectionPointEvidence2,
    certified_direction: Option<BezierAlgebraicChordAxisDirection2>,
    certified_circle_transverse_endpoints: [bool; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let endpoint_equality = match certified_direction {
        Some(_) => Classification::Decided(false),
        None => from.same_point(to, policy),
    };
    match endpoint_equality {
        Classification::Decided(true) => Ok(Classification::Decided(())),
        Classification::Decided(false) => {
            let chord = match certified_direction {
                Some(direction) => {
                    crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
                        from.clone(),
                        to.clone(),
                        direction,
                        policy,
                    )
                }
                None => {
                    match crate::BezierAlgebraicChord2::try_new(from.clone(), to.clone(), policy)? {
                        Classification::Decided(chord) => chord,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            };
            let chord = chord
                .with_certified_circle_transverse_endpoints(certified_circle_transverse_endpoints);
            fragments.push(retained_chord_or_exact_line_fragment(chord)?);
            Ok(Classification::Decided(()))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn append_exact_axis_aligned_algebraic_round_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactAxisAlignedAlgebraicOffsetSpan2,
    next: &ExactAxisAlignedAlgebraicOffsetSpan2,
    distance: &Real,
    sweep_kind: crate::arc_bezier::ArcSweepKind,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let clockwise = match real_sign(distance, policy) {
        Some(RealSign::Positive) => true,
        Some(RealSign::Negative) => false,
        Some(RealSign::Zero) => return Ok(Classification::Decided(())),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };

    if let (Some(start), Some(end), Some(center)) = (
        previous.offset_end.as_exact(),
        next.offset_start.as_exact(),
        previous.source.end().as_exact(),
    ) {
        let radius_squared = distance * distance;
        let arc = CircularArc2::new_with_certified_radius_and_sweep(
            start.clone(),
            end.clone(),
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
        let circular_conic = Arc::new(crate::rational_bezier::RationalQuadraticCircle2 {
            center: center.clone(),
            radius_squared,
            tangent_contacts: None,
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

    let semicircle = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
        previous.source.end(),
        previous.direction.left_cardinal_normal(),
        distance.clone(),
        clockwise,
        policy,
    )? {
        Classification::Decided(Some(semicircle)) => semicircle,
        Classification::Decided(None) => return Ok(Classification::Decided(())),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_parameter = match sweep_kind {
        crate::arc_bezier::ArcSweepKind::Minor => (Real::one() / Real::from(2_i8))?,
        crate::arc_bezier::ArcSweepKind::Semicircle => Real::one(),
        crate::arc_bezier::ArcSweepKind::Major | crate::arc_bezier::ArcSweepKind::FullCircle => {
            return Err(CurveError::Topology(
                "axis-aligned round join requested more than one semicircle".into(),
            ));
        }
    };
    let fragment = match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
        semicircle,
        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(end_parameter),
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
                    "selected algebraic round join missed its certified endpoint".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    fragments.push(BezierSplitFragment2::AlgebraicCuspSemicircle(fragment));
    Ok(Classification::Decided(()))
}

fn exact_axis_aligned_algebraic_miter_point(
    previous: &ExactAxisAlignedAlgebraicOffsetSpan2,
    next: &ExactAxisAlignedAlgebraicOffsetSpan2,
    distance: &Real,
    limit: Option<&Real>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<crate::RationalBezierIntersectionPointEvidence2>>> {
    let previous_tangent = previous.direction.unit_tangent();
    let next_tangent = next.direction.unit_tangent();
    let denominator = offset_vector_cross(&previous_tangent, &next_tangent);
    match real_sign(&denominator, policy) {
        Some(RealSign::Positive | RealSign::Negative) => {}
        Some(RealSign::Zero) => {
            return Ok(Classification::Decided(None));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let miter_delta = (
        &previous.normal_offset.0 + &next.normal_offset.0,
        &previous.normal_offset.1 + &next.normal_offset.1,
    );
    if let Some(limit) = limit {
        let miter_distance_squared =
            &miter_delta.0 * &miter_delta.0 + &miter_delta.1 * &miter_delta.1;
        let maximum_squared = distance * distance * limit * limit;
        match compare_reals(&miter_distance_squared, &maximum_squared, policy) {
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {}
            Some(std::cmp::Ordering::Greater) => {
                return Ok(Classification::Decided(None));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }
    let miter = match crate::BezierAlgebraicChord2::translated_endpoint(
        previous.source.end(),
        &miter_delta.0,
        &miter_delta.1,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(Some(miter)))
}

enum ExactAxisAlignedAlgebraicJoin2 {
    Shared,
    Miter(crate::RationalBezierIntersectionPointEvidence2),
    Line,
    Round(crate::arc_bezier::ArcSweepKind),
}

enum ExactAxisAlignedAlgebraicOffsetLoop2 {
    Inapplicable,
    Removed,
    Boundary(CurveRegionBoundaryLoop2),
    ErodedBoundaries(Vec<(CurveRegionBoundaryLoop2, bool)>),
}

#[derive(Clone, Copy)]
struct ExactCardinalConvexFragment2 {
    start_tangent: BezierAlgebraicChordAxisDirection2,
    end_tangent: BezierAlgebraicChordAxisDirection2,
    quarter_turns: u8,
}

const fn cardinal_direction_index(direction: BezierAlgebraicChordAxisDirection2) -> u8 {
    match direction {
        BezierAlgebraicChordAxisDirection2::PositiveX => 0,
        BezierAlgebraicChordAxisDirection2::PositiveY => 1,
        BezierAlgebraicChordAxisDirection2::NegativeX => 2,
        BezierAlgebraicChordAxisDirection2::NegativeY => 3,
    }
}

fn cardinal_direction_from_delta(
    x: &Real,
    y: &Real,
    policy: &CurveContext,
) -> Classification<Option<BezierAlgebraicChordAxisDirection2>> {
    let Some(x_sign) = real_sign(x, policy) else {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    };
    let Some(y_sign) = real_sign(y, policy) else {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    };
    Classification::Decided(match (x_sign, y_sign) {
        (RealSign::Positive, RealSign::Zero) => Some(BezierAlgebraicChordAxisDirection2::PositiveX),
        (RealSign::Negative, RealSign::Zero) => Some(BezierAlgebraicChordAxisDirection2::NegativeX),
        (RealSign::Zero, RealSign::Positive) => Some(BezierAlgebraicChordAxisDirection2::PositiveY),
        (RealSign::Zero, RealSign::Negative) => Some(BezierAlgebraicChordAxisDirection2::NegativeY),
        (RealSign::Zero, RealSign::Zero)
        | (RealSign::Positive | RealSign::Negative, RealSign::Positive | RealSign::Negative) => {
            None
        }
    })
}

fn exact_cardinal_convex_fragment(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<ExactCardinalConvexFragment2>>> {
    let line_certificate = |line: &LineSeg2| {
        let (dx, dy) = line.delta();
        cardinal_direction_from_delta(&dx, &dy, policy).map(|direction| {
            direction.map(|direction| ExactCardinalConvexFragment2 {
                start_tangent: direction,
                end_tangent: direction,
                quarter_turns: 0,
            })
        })
    };
    match fragment {
        BezierSplitFragment2::AlgebraicChord(chord) => {
            chord.axis_direction(policy).map(|classification| {
                classification.map(|direction| {
                    direction.map(|direction| ExactCardinalConvexFragment2 {
                        start_tangent: direction,
                        end_tangent: direction,
                        quarter_turns: 0,
                    })
                })
            })
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            if fragment.semicircle().is_clockwise() ^ fragment.is_reversed() {
                return Ok(Classification::Decided(None));
            }
            let start = match fragment.cardinal_endpoint_tangent_direction(true, policy)? {
                Classification::Decided(Some(direction)) => direction,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let end = match fragment.cardinal_endpoint_tangent_direction(false, policy)? {
                Classification::Decided(Some(direction)) => direction,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let quarter_turns =
                (cardinal_direction_index(end) + 4 - cardinal_direction_index(start)) % 4;
            Ok(Classification::Decided(
                matches!(quarter_turns, 1 | 2).then_some(ExactCardinalConvexFragment2 {
                    start_tangent: start,
                    end_tangent: end,
                    quarter_turns,
                }),
            ))
        }
        BezierSplitFragment2::Materialized { curve, .. } => {
            match materialized_native_subcurve_segment(curve, policy)? {
                Classification::Decided(Segment2::Line(line)) => Ok(line_certificate(&line)),
                Classification::Decided(Segment2::Arc(arc)) => {
                    if arc.is_clockwise() {
                        return Ok(Classification::Decided(None));
                    }
                    let sweep = match crate::arc_bezier::classify_sweep_with_policy(&arc, policy) {
                        Ok(Classification::Decided(sweep)) => sweep,
                        Ok(Classification::Uncertain(reason)) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                        Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
                        Err(ExactCurveError::Blocked(blocker)) => {
                            return Ok(Classification::Uncertain(blocker.reason()));
                        }
                    };
                    if !matches!(
                        sweep,
                        crate::arc_bezier::ArcSweepKind::Minor
                            | crate::arc_bezier::ArcSweepKind::Semicircle
                    ) {
                        return Ok(Classification::Decided(None));
                    }
                    let start_radius = arc.start().delta_from(arc.center());
                    let end_radius = arc.end().delta_from(arc.center());
                    let start = match cardinal_direction_from_delta(
                        &-start_radius.1,
                        &start_radius.0,
                        policy,
                    ) {
                        Classification::Decided(Some(direction)) => direction,
                        Classification::Decided(None) => {
                            return Ok(Classification::Decided(None));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let end = match cardinal_direction_from_delta(
                        &-end_radius.1,
                        &end_radius.0,
                        policy,
                    ) {
                        Classification::Decided(Some(direction)) => direction,
                        Classification::Decided(None) => {
                            return Ok(Classification::Decided(None));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let quarter_turns =
                        (cardinal_direction_index(end) + 4 - cardinal_direction_index(start)) % 4;
                    let expected_turns = match sweep {
                        crate::arc_bezier::ArcSweepKind::Minor => 1,
                        crate::arc_bezier::ArcSweepKind::Semicircle => 2,
                        crate::arc_bezier::ArcSweepKind::Major
                        | crate::arc_bezier::ArcSweepKind::FullCircle => unreachable!(),
                    };
                    Ok(Classification::Decided(
                        (quarter_turns == expected_turns).then_some(ExactCardinalConvexFragment2 {
                            start_tangent: start,
                            end_tangent: end,
                            quarter_turns,
                        }),
                    ))
                }
                Classification::Uncertain(UncertaintyReason::Unsupported) => {
                    Ok(Classification::Decided(None))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        }
        BezierSplitFragment2::AnalyticParallel(_)
        | BezierSplitFragment2::AlgebraicEndpointImages { .. }
        | BezierSplitFragment2::SelectedFiber(_)
        | BezierSplitFragment2::Unresolved { .. } => Ok(Classification::Decided(None)),
    }
}

/// Certifies one already-regularized filled-left loop as a cardinal convex
/// boundary. Each admitted curve contributes zero, one, or two positive
/// quarter turns, every join contributes at most one, and exactly one full
/// positive turn is required. No coordinate sampling or approximate tangent
/// construction participates in the certificate.
fn exact_cardinal_convex_filled_left_loop(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let mut certificates = Vec::with_capacity(boundary_loop.len());
    for fragment in boundary_loop.fragments() {
        match exact_cardinal_convex_fragment(fragment, policy)? {
            Classification::Decided(Some(certificate)) => certificates.push(certificate),
            Classification::Decided(None) => return Ok(Classification::Decided(false)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    if certificates.is_empty() {
        return Ok(Classification::Decided(false));
    }
    let mut quarter_turns = certificates
        .iter()
        .map(|certificate| usize::from(certificate.quarter_turns))
        .sum::<usize>();
    for index in 0..certificates.len() {
        let previous = certificates[index].end_tangent;
        let next = certificates[(index + 1) % certificates.len()].start_tangent;
        let join_turn =
            (cardinal_direction_index(next) + 4 - cardinal_direction_index(previous)) % 4;
        if join_turn > 1 {
            return Ok(Classification::Decided(false));
        }
        quarter_turns += usize::from(join_turn);
    }
    Ok(Classification::Decided(quarter_turns == 4))
}

fn axis_aligned_loop_is_convex_inward(
    spans: &[ExactAxisAlignedAlgebraicFiber2],
    distance: &Real,
    policy: &CurveContext,
) -> Classification<bool> {
    let mut orientation = None;
    for span_index in 0..spans.len() {
        let previous = spans[span_index].direction.unit_tangent();
        let next = spans[(span_index + 1) % spans.len()]
            .direction
            .unit_tangent();
        if offset_vectors_are_structurally_opposite(&previous, &next) {
            return Classification::Decided(false);
        }
        let turn = match real_sign(&offset_vector_cross(&previous, &next), policy) {
            Some(RealSign::Zero) => continue,
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        };
        match orientation {
            Some(existing) if existing != turn => return Classification::Decided(false),
            Some(_) => {}
            None => orientation = Some(turn),
        }
    }
    let Some(orientation) = orientation else {
        return Classification::Decided(false);
    };
    match real_sign(distance, policy) {
        Some(RealSign::Positive) => Classification::Decided(orientation == RealSign::Positive),
        Some(RealSign::Negative) => Classification::Decided(orientation == RealSign::Negative),
        Some(RealSign::Zero) => Classification::Decided(false),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn axis_aligned_offset_has_reversed_span(
    spans: &[ExactAxisAlignedAlgebraicOffsetSpan2],
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    for span in spans {
        // Offset construction preserves the certified constant coordinate.
        // Compare only the original traversal axis: rebuilding a general
        // chord would repeat both endpoint-field predicates, allocate a
        // carrier, and discard the direction certificate already in `span`.
        match crate::BezierAlgebraicChord2::point_axis_order(
            &span.offset_start,
            &span.offset_end,
            span.direction.axis(),
            policy,
        )? {
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Ok(Classification::Decided(true));
            }
            Classification::Decided(order)
                if (order == std::cmp::Ordering::Less)
                    == matches!(
                        span.direction,
                        BezierAlgebraicChordAxisDirection2::PositiveX
                            | BezierAlgebraicChordAxisDirection2::PositiveY
                    ) => {}
            Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                return Ok(Classification::Decided(true));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(false))
}

struct ExactAxisAlignedAlgebraicExpandedBox2 {
    minimum_x: crate::RationalBezierIntersectionPointEvidence2,
    maximum_x: crate::RationalBezierIntersectionPointEvidence2,
    minimum_y: crate::RationalBezierIntersectionPointEvidence2,
    maximum_y: crate::RationalBezierIntersectionPointEvidence2,
}

#[derive(Clone, Copy)]
struct ExactAxisAlignedAlgebraicGridEdge2 {
    start: usize,
    end: usize,
    direction: usize,
}

fn translated_algebraic_offset_endpoint(
    endpoint: &crate::RationalBezierIntersectionPointEvidence2,
    delta_x: &Real,
    delta_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<crate::RationalBezierIntersectionPointEvidence2>> {
    crate::BezierAlgebraicChord2::translated_endpoint(endpoint, delta_x, delta_y, policy)
}

fn exact_axis_aligned_expanded_source_box(
    span: &ExactAxisAlignedAlgebraicFiber2,
    radius: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactAxisAlignedAlgebraicExpandedBox2>> {
    let source_start = span.chord.start();
    let source_end = span.chord.end();
    let (minimum_x, maximum_x, minimum_y, maximum_y) = match span.direction {
        BezierAlgebraicChordAxisDirection2::PositiveX => {
            (source_start, source_end, source_start, source_start)
        }
        BezierAlgebraicChordAxisDirection2::NegativeX => {
            (source_end, source_start, source_start, source_start)
        }
        BezierAlgebraicChordAxisDirection2::PositiveY => {
            (source_start, source_start, source_start, source_end)
        }
        BezierAlgebraicChordAxisDirection2::NegativeY => {
            (source_start, source_start, source_end, source_start)
        }
    };
    let zero = Real::zero();
    let negative_radius = -radius;
    let translated = |endpoint, delta_x: &Real, delta_y: &Real| {
        translated_algebraic_offset_endpoint(endpoint, delta_x, delta_y, policy)
    };
    let minimum_x = match translated(minimum_x, &negative_radius, &zero)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let maximum_x = match translated(maximum_x, radius, &zero)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let minimum_y = match translated(minimum_y, &zero, &negative_radius)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let maximum_y = match translated(maximum_y, &zero, radius)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(
        ExactAxisAlignedAlgebraicExpandedBox2 {
            minimum_x,
            maximum_x,
            minimum_y,
            maximum_y,
        },
    ))
}

fn extend_axis_aligned_algebraic_chord(
    chord: &crate::BezierAlgebraicChord2,
    direction: BezierAlgebraicChordAxisDirection2,
    delta_x: &Real,
    delta_y: &Real,
    extent: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<crate::BezierAlgebraicChord2>> {
    let (start_delta_x, start_delta_y, end_delta_x, end_delta_y) = match direction {
        BezierAlgebraicChordAxisDirection2::PositiveX => (
            delta_x - extent,
            delta_y.clone(),
            delta_x + extent,
            delta_y.clone(),
        ),
        BezierAlgebraicChordAxisDirection2::NegativeX => (
            delta_x + extent,
            delta_y.clone(),
            delta_x - extent,
            delta_y.clone(),
        ),
        BezierAlgebraicChordAxisDirection2::PositiveY => (
            delta_x.clone(),
            delta_y - extent,
            delta_x.clone(),
            delta_y + extent,
        ),
        BezierAlgebraicChordAxisDirection2::NegativeY => (
            delta_x.clone(),
            delta_y + extent,
            delta_x.clone(),
            delta_y - extent,
        ),
    };
    let start = match translated_algebraic_offset_endpoint(
        chord.start(),
        &start_delta_x,
        &start_delta_y,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let end = match translated_algebraic_offset_endpoint(
        chord.end(),
        &end_delta_x,
        &end_delta_y,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(
        crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
            start, end, direction, policy,
        ),
    ))
}

fn sort_dedup_axis_aligned_algebraic_fibers(
    fibers: Vec<ExactAxisAlignedAlgebraicFiber2>,
    axis: Axis2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactAxisAlignedAlgebraicFiber2>>> {
    let mut sorted: Vec<ExactAxisAlignedAlgebraicFiber2> = Vec::with_capacity(fibers.len());
    for fiber in fibers {
        let mut lower = 0;
        let mut upper = sorted.len();
        let mut duplicate = false;
        while lower < upper {
            let middle = lower + (upper - lower) / 2;
            match fiber
                .chord
                .axis_coordinate_order(&sorted[middle].chord, axis, policy)?
            {
                Classification::Decided(std::cmp::Ordering::Less) => {
                    upper = middle;
                }
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    duplicate = true;
                    break;
                }
                Classification::Decided(std::cmp::Ordering::Greater) => {
                    lower = middle + 1;
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if !duplicate {
            sorted.insert(lower, fiber);
        }
    }
    Ok(Classification::Decided(sorted))
}

fn exact_axis_aligned_algebraic_erosion_coordinates(
    sources: &[ExactAxisAlignedAlgebraicFiber2],
    axis: Axis2,
    radius: &Real,
    extent: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactAxisAlignedAlgebraicFiber2>>> {
    if sources.is_empty() {
        return Ok(Classification::Decided(Vec::new()));
    }
    let zero = Real::zero();
    let negative_radius = -radius;
    let mut candidates = Vec::with_capacity(sources.len().saturating_mul(3));
    for source in sources {
        let translated = match axis {
            Axis2::X => [(&negative_radius, &zero), (&zero, &zero), (radius, &zero)],
            Axis2::Y => [(&zero, &negative_radius), (&zero, &zero), (&zero, radius)],
        };
        let direction = source.direction;
        for (delta_x, delta_y) in translated {
            match extend_axis_aligned_algebraic_chord(
                &source.chord,
                direction,
                delta_x,
                delta_y,
                extent,
                policy,
            )? {
                Classification::Decided(chord) => {
                    candidates.push(ExactAxisAlignedAlgebraicFiber2 { chord, direction });
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }
    let candidates = match sort_dedup_axis_aligned_algebraic_fibers(candidates, axis, policy)? {
        Classification::Decided(candidates) => candidates,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let minimum = &sources[0].chord;
    let maximum = &sources[sources.len() - 1].chord;
    let mut retained = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        let above_minimum = minimum.axis_coordinate_order(&candidate.chord, axis, policy)?;
        let below_maximum = candidate
            .chord
            .axis_coordinate_order(maximum, axis, policy)?;
        match (above_minimum, below_maximum) {
            (
                Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
                Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            ) => retained.push(candidate),
            (Classification::Decided(_), Classification::Decided(_)) => {}
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(retained))
}

fn exact_axis_aligned_grid_coordinate_position(
    point: &crate::RationalBezierIntersectionPointEvidence2,
    axis: Axis2,
    fibers: &[ExactAxisAlignedAlgebraicFiber2],
    policy: &CurveContext,
) -> CurveResult<Classification<(usize, bool)>> {
    let mut lower = 0;
    let mut upper = fibers.len();
    while lower < upper {
        let middle = lower + (upper - lower) / 2;
        match crate::BezierAlgebraicChord2::point_axis_order(
            point,
            fibers[middle].chord.start(),
            axis,
            policy,
        )? {
            Classification::Decided(std::cmp::Ordering::Less) => upper = middle,
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Ok(Classification::Decided((middle, true)));
            }
            Classification::Decided(std::cmp::Ordering::Greater) => lower = middle + 1,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided((lower, false)))
}

fn exact_axis_aligned_grid_coordinate_index(
    point: &crate::RationalBezierIntersectionPointEvidence2,
    axis: Axis2,
    fibers: &[ExactAxisAlignedAlgebraicFiber2],
    policy: &CurveContext,
) -> CurveResult<Classification<usize>> {
    match exact_axis_aligned_grid_coordinate_position(point, axis, fibers, policy)? {
        Classification::Decided((index, true)) => Ok(Classification::Decided(index)),
        Classification::Decided((_, false)) => Err(CurveError::Topology(
            "orthogonal source coordinate was absent from its exact arrangement".into(),
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn exact_axis_aligned_grid_cell_interval(
    minimum: &crate::RationalBezierIntersectionPointEvidence2,
    maximum: &crate::RationalBezierIntersectionPointEvidence2,
    axis: Axis2,
    fibers: &[ExactAxisAlignedAlgebraicFiber2],
    policy: &CurveContext,
) -> CurveResult<Classification<std::ops::Range<usize>>> {
    let (minimum, minimum_is_grid_coordinate) =
        match exact_axis_aligned_grid_coordinate_position(minimum, axis, fibers, policy)? {
            Classification::Decided(position) => position,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    let (maximum, maximum_is_grid_coordinate) =
        match exact_axis_aligned_grid_coordinate_position(maximum, axis, fibers, policy)? {
            Classification::Decided(position) => position,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if (!minimum_is_grid_coordinate && minimum != 0)
        || (!maximum_is_grid_coordinate && maximum != fibers.len())
    {
        return Err(CurveError::Topology(
            "orthogonal erosion bound fell inside an unpartitioned grid cell".into(),
        ));
    }
    let cell_count = fibers.len().saturating_sub(1);
    let minimum = minimum.min(cell_count);
    let maximum = maximum.min(cell_count);
    if minimum > maximum {
        return Err(CurveError::Topology(
            "orthogonal erosion bounds were reversed".into(),
        ));
    }
    Ok(Classification::Decided(minimum..maximum))
}

fn exact_axis_aligned_grid_point(
    cache: &mut [Option<crate::RationalBezierIntersectionPointEvidence2>],
    vertex: usize,
    x_count: usize,
    x_fibers: &[ExactAxisAlignedAlgebraicFiber2],
    y_fibers: &[ExactAxisAlignedAlgebraicFiber2],
    policy: &CurveContext,
) -> CurveResult<Classification<crate::RationalBezierIntersectionPointEvidence2>> {
    if let Some(point) = &cache[vertex] {
        return Ok(Classification::Decided(point.clone()));
    }
    let x = vertex % x_count;
    let y = vertex / x_count;
    let point = x_fibers[x].chord.certified_axis_aligned_crossing_point(
        &y_fibers[y].chord,
        x_fibers[x].direction,
        y_fibers[y].direction,
        policy,
    )?;
    cache[vertex] = Some(point.clone());
    Ok(Classification::Decided(point))
}

fn push_exact_axis_aligned_grid_edge(
    edges: &mut Vec<ExactAxisAlignedAlgebraicGridEdge2>,
    outgoing: &mut [[Option<usize>; 4]],
    start: usize,
    end: usize,
    direction: usize,
) -> CurveResult<()> {
    if outgoing[start][direction].is_some() {
        return Err(CurveError::Topology(
            "orthogonal erosion emitted a duplicate directed grid edge".into(),
        ));
    }
    let index = edges.len();
    edges.push(ExactAxisAlignedAlgebraicGridEdge2 {
        start,
        end,
        direction,
    });
    outgoing[start][direction] = Some(index);
    Ok(())
}

/// Erodes one orthogonal retained loop through its exact finite cell arrangement.
///
/// Source coordinates and their `radius` translations partition the plane into
/// cells on which winding and L-infinity boundary distance are constant. The
/// compact occupancy bitmap therefore proves every retained face. Exposed grid
/// runs are traced with material on the left and emitted as subchords of the
/// shared coordinate fibers, preserving selected algebraic endpoint fields.
fn exact_axis_aligned_algebraic_erosion(
    source_loop: &ExactAxisAlignedAlgebraicErosionSource2,
    spans: &[ExactAxisAlignedAlgebraicFiber2],
    distance: &Real,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<(CurveRegionBoundaryLoop2, bool)>>> {
    let radius = match real_sign(distance, policy) {
        Some(RealSign::Positive) => distance.clone(),
        Some(RealSign::Negative) => -distance,
        Some(RealSign::Zero) => return Ok(Classification::Decided(Vec::new())),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    let mut expanded_boxes = Vec::with_capacity(spans.len());
    for span in spans {
        let expanded = match exact_axis_aligned_expanded_source_box(span, &radius, policy)? {
            Classification::Decided(expanded) => expanded,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        expanded_boxes.push(expanded);
    }
    let source_bounds = &source_loop.source_bounds;
    let width = source_bounds.max().x() - source_bounds.min().x();
    let height = source_bounds.max().y() - source_bounds.min().y();
    let extent = width + height + Real::from(4_u8) * &radius + Real::one();
    let x_fibers = match exact_axis_aligned_algebraic_erosion_coordinates(
        &source_loop.source_x_fibers,
        Axis2::X,
        &radius,
        &extent,
        policy,
    )? {
        Classification::Decided(fibers) => fibers,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let y_fibers = match exact_axis_aligned_algebraic_erosion_coordinates(
        &source_loop.source_y_fibers,
        Axis2::Y,
        &radius,
        &extent,
        policy,
    )? {
        Classification::Decided(fibers) => fibers,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if x_fibers.len() < 2 || y_fibers.len() < 2 {
        return Ok(Classification::Decided(Vec::new()));
    }

    let cell_width = x_fibers.len() - 1;
    let cell_height = y_fibers.len() - 1;
    let cell_count = cell_width.saturating_mul(cell_height);
    let mut winding = vec![0_i32; cell_count];
    for span in spans {
        let delta = match span.direction {
            BezierAlgebraicChordAxisDirection2::PositiveY => 1,
            BezierAlgebraicChordAxisDirection2::NegativeY => -1,
            BezierAlgebraicChordAxisDirection2::PositiveX
            | BezierAlgebraicChordAxisDirection2::NegativeX => continue,
        };
        let x_limit = match exact_axis_aligned_grid_coordinate_index(
            span.chord.start(),
            Axis2::X,
            &x_fibers,
            policy,
        )? {
            Classification::Decided(index) => index,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (lower, upper) = if delta > 0 {
            (span.chord.start(), span.chord.end())
        } else {
            (span.chord.end(), span.chord.start())
        };
        let lower =
            match exact_axis_aligned_grid_coordinate_index(lower, Axis2::Y, &y_fibers, policy)? {
                Classification::Decided(index) => index,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let upper =
            match exact_axis_aligned_grid_coordinate_index(upper, Axis2::Y, &y_fibers, policy)? {
                Classification::Decided(index) => index,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        if lower >= upper || x_limit > cell_width {
            return Err(CurveError::Topology(
                "orthogonal winding span did not map to an ordered grid interval".into(),
            ));
        }
        for y in lower..upper {
            for x in 0..x_limit {
                winding[y * cell_width + x] += delta;
            }
        }
    }
    let mut occupied = winding
        .into_iter()
        .map(|winding| match fill_rule {
            FillRule::EvenOdd => winding.rem_euclid(2) != 0,
            FillRule::NonZero => winding != 0,
        })
        .collect::<Vec<_>>();
    for bounds in &expanded_boxes {
        let x = match exact_axis_aligned_grid_cell_interval(
            &bounds.minimum_x,
            &bounds.maximum_x,
            Axis2::X,
            &x_fibers,
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let y = match exact_axis_aligned_grid_cell_interval(
            &bounds.minimum_y,
            &bounds.maximum_y,
            Axis2::Y,
            &y_fibers,
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for y in y {
            for x in x.clone() {
                occupied[y * cell_width + x] = false;
            }
        }
    }
    if !occupied.iter().any(|cell| *cell) {
        return Ok(Classification::Decided(Vec::new()));
    }
    let x_count = x_fibers.len();
    let vertex = |x: usize, y: usize| y * x_count + x;
    let mut edges = Vec::new();
    let mut outgoing = vec![[None; 4]; x_count.saturating_mul(y_fibers.len())];
    let is_occupied = |x: usize, y: usize| occupied[y * cell_width + x];
    for y in 0..cell_height {
        for x in 0..cell_width {
            if !is_occupied(x, y) {
                continue;
            }
            if y == 0 || !is_occupied(x, y - 1) {
                push_exact_axis_aligned_grid_edge(
                    &mut edges,
                    &mut outgoing,
                    vertex(x, y),
                    vertex(x + 1, y),
                    0,
                )?;
            }
            if x + 1 == cell_width || !is_occupied(x + 1, y) {
                push_exact_axis_aligned_grid_edge(
                    &mut edges,
                    &mut outgoing,
                    vertex(x + 1, y),
                    vertex(x + 1, y + 1),
                    1,
                )?;
            }
            if y + 1 == cell_height || !is_occupied(x, y + 1) {
                push_exact_axis_aligned_grid_edge(
                    &mut edges,
                    &mut outgoing,
                    vertex(x + 1, y + 1),
                    vertex(x, y + 1),
                    2,
                )?;
            }
            if x == 0 || !is_occupied(x - 1, y) {
                push_exact_axis_aligned_grid_edge(
                    &mut edges,
                    &mut outgoing,
                    vertex(x, y + 1),
                    vertex(x, y),
                    3,
                )?;
            }
        }
    }

    let mut used = vec![false; edges.len()];
    let mut point_cache = vec![None; x_count.saturating_mul(y_fibers.len())];
    let mut boundaries = Vec::new();
    let mut arrangement_fragment_index = 0;
    let mut cycle = Vec::new();
    let mut corners = Vec::new();
    for first_edge in 0..edges.len() {
        if used[first_edge] {
            continue;
        }
        let cycle_start = edges[first_edge].start;
        cycle.clear();
        let mut current = first_edge;
        loop {
            if used[current] {
                return Err(CurveError::Topology(
                    "orthogonal erosion boundary revisited an edge before closure".into(),
                ));
            }
            used[current] = true;
            cycle.push(current);
            let edge = edges[current];
            if edge.end == cycle_start {
                break;
            }
            let priorities = [
                (edge.direction + 1) % 4,
                edge.direction,
                (edge.direction + 3) % 4,
                (edge.direction + 2) % 4,
            ];
            let Some(next) = priorities
                .into_iter()
                .find_map(|direction| outgoing[edge.end][direction].filter(|next| !used[*next]))
            else {
                return Err(CurveError::Topology(
                    "orthogonal erosion boundary did not close".into(),
                ));
            };
            current = next;
            if cycle.len() > edges.len() {
                return Err(CurveError::Topology(
                    "orthogonal erosion boundary exceeded its edge inventory".into(),
                ));
            }
        }

        let mut turn_sum = 0_i32;
        corners.clear();
        for index in 0..cycle.len() {
            let previous = edges[cycle[(index + cycle.len() - 1) % cycle.len()]];
            let current = edges[cycle[index]];
            let turn = (current.direction + 4 - previous.direction) % 4;
            match turn {
                0 => {}
                1 => turn_sum += 1,
                3 => turn_sum -= 1,
                2 => {
                    return Err(CurveError::Topology(
                        "orthogonal erosion boundary made a reversing turn".into(),
                    ));
                }
                _ => unreachable!(),
            }
            if previous.direction != current.direction {
                corners.push(cycle[index]);
            }
        }
        if corners.len() < 4 || turn_sum == 0 {
            return Err(CurveError::Topology(
                "orthogonal erosion boundary did not form an oriented area cycle".into(),
            ));
        }

        let mut fragments = Vec::with_capacity(corners.len());
        for index in 0..corners.len() {
            let run = edges[corners[index]];
            let next = edges[corners[(index + 1) % corners.len()]];
            let start = match exact_axis_aligned_grid_point(
                &mut point_cache,
                run.start,
                x_count,
                &x_fibers,
                &y_fibers,
                policy,
            )? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let end = match exact_axis_aligned_grid_point(
                &mut point_cache,
                next.start,
                x_count,
                &x_fibers,
                &y_fibers,
                policy,
            )? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let support = match run.direction {
                0 | 2 => &y_fibers[run.start / x_count],
                1 | 3 => &x_fibers[run.start % x_count],
                _ => unreachable!(),
            };
            let support_direction = support.direction;
            let forward_on_support = matches!(
                (support_direction, run.direction),
                (BezierAlgebraicChordAxisDirection2::PositiveX, 0)
                    | (BezierAlgebraicChordAxisDirection2::PositiveY, 1)
                    | (BezierAlgebraicChordAxisDirection2::NegativeX, 2)
                    | (BezierAlgebraicChordAxisDirection2::NegativeY, 3)
            );
            let chord = match support.chord.subchord_between_certified_points(
                start,
                end,
                forward_on_support,
                policy,
            )? {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            fragments.push(retained_chord_or_exact_line_fragment(chord)?);
        }
        let arrangement_sources = (0..fragments.len())
            .map(|source_fragment_index| {
                let source = CurveRegionFragmentSource2::new(
                    arrangement_fragment_index,
                    arrangement_fragment_index,
                    source_fragment_index,
                );
                arrangement_fragment_index += 1;
                source
            })
            .collect();
        boundaries.push((
            CurveRegionBoundaryLoop2::try_new_with_arrangement_sources(
                fragments,
                arrangement_sources,
                policy,
            )?,
            turn_sum > 0,
        ));
    }
    Ok(Classification::Decided(boundaries))
}

fn exact_axis_aligned_algebraic_join(
    previous: &ExactAxisAlignedAlgebraicOffsetSpan2,
    next: &ExactAxisAlignedAlgebraicOffsetSpan2,
    distance: &Real,
    style: &OffsetCornerStyle2,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactAxisAlignedAlgebraicJoin2>> {
    match previous.offset_end.same_point(&next.offset_start, policy) {
        Classification::Decided(true) => {
            return Ok(Classification::Decided(
                ExactAxisAlignedAlgebraicJoin2::Shared,
            ));
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
    match previous
        .source
        .end()
        .same_point(next.source.start(), policy)
    {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Err(CurveError::DisconnectedCurvePath),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let previous_tangent = previous.direction.unit_tangent();
    let next_tangent = next.direction.unit_tangent();
    let opposite_tangents =
        offset_vectors_are_structurally_opposite(&previous_tangent, &next_tangent);
    let turn = if opposite_tangents {
        Real::zero()
    } else {
        offset_vector_cross(&previous_tangent, &next_tangent)
    };
    let inward = match real_sign(&(turn * distance), policy) {
        Some(RealSign::Positive) => true,
        Some(RealSign::Negative | RealSign::Zero) => false,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    if !inward {
        match style {
            OffsetCornerStyle2::Round => {
                return Ok(Classification::Decided(
                    ExactAxisAlignedAlgebraicJoin2::Round(if opposite_tangents {
                        crate::arc_bezier::ArcSweepKind::Semicircle
                    } else {
                        crate::arc_bezier::ArcSweepKind::Minor
                    }),
                ));
            }
            OffsetCornerStyle2::Bevel => {
                return Ok(Classification::Decided(
                    ExactAxisAlignedAlgebraicJoin2::Line,
                ));
            }
            OffsetCornerStyle2::Miter { .. } => {}
        }
    }
    let limit = match style {
        OffsetCornerStyle2::Miter { limit } if !inward => Some(limit),
        OffsetCornerStyle2::Round
        | OffsetCornerStyle2::Bevel
        | OffsetCornerStyle2::Miter { .. } => None,
    };
    exact_axis_aligned_algebraic_miter_point(previous, next, distance, limit, policy).map(
        |classification| {
            classification.map(|miter| {
                miter.map_or(
                    ExactAxisAlignedAlgebraicJoin2::Line,
                    ExactAxisAlignedAlgebraicJoin2::Miter,
                )
            })
        },
    )
}

fn exact_axis_aligned_algebraic_source_loop(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Arc<ExactAxisAlignedAlgebraicSourceLoop2>>>> {
    if !boundary_loop
        .fragments()
        .iter()
        .any(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
    {
        return Ok(Classification::Decided(None));
    }
    let mut spans = Vec::with_capacity(boundary_loop.len());
    for fragment in boundary_loop.fragments() {
        let chord = match fragment {
            BezierSplitFragment2::AlgebraicChord(chord) => chord.clone(),
            BezierSplitFragment2::Materialized { curve, .. } => {
                let line = match materialized_native_subcurve_segment(curve, policy)? {
                    Classification::Decided(Segment2::Line(line)) => line,
                    Classification::Decided(Segment2::Arc(_))
                    | Classification::Uncertain(UncertaintyReason::Unsupported) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match crate::BezierAlgebraicChord2::try_new(
                    crate::RationalBezierIntersectionPointEvidence2::Exact(line.start().clone()),
                    crate::RationalBezierIntersectionPointEvidence2::Exact(line.end().clone()),
                    policy,
                )? {
                    Classification::Decided(chord) => chord,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicEndpointImages { .. }
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::SelectedFiber(_)
            | BezierSplitFragment2::Unresolved { .. } => {
                return Ok(Classification::Decided(None));
            }
        };
        let direction = match chord.axis_direction(policy)? {
            Classification::Decided(Some(direction)) => direction,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        spans.push(ExactAxisAlignedAlgebraicFiber2 { chord, direction });
    }
    if spans.is_empty() {
        return Ok(Classification::Decided(None));
    }
    Ok(Classification::Decided(Some(Arc::new(
        ExactAxisAlignedAlgebraicSourceLoop2 {
            spans: Arc::from(spans),
            erosion_source: OnceLock::new(),
        },
    ))))
}

impl ExactAxisAlignedAlgebraicSourceLoop2 {
    fn erosion_source(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Arc<ExactAxisAlignedAlgebraicErosionSource2>>> {
        if let Some((cached_policy, certainty, source)) = self.erosion_source.get()
            && cached_policy == policy
        {
            if *certainty == CurveCertainty::Approximate512Consumed {
                policy.observe_approximate_512();
            }
            return Ok(Classification::Decided(Arc::clone(source)));
        }
        let outcome = resolve_certified_value(
            policy,
            |attempt| -> CurveResult<Classification<Arc<ExactAxisAlignedAlgebraicErosionSource2>>> {
                let mut source_bounds: Option<Aabb2> = None;
                let mut source_x_fibers = Vec::new();
                let mut source_y_fibers = Vec::new();
                for source in self.spans.iter() {
                    let bounds = match source.chord.conservative_bounds_refined(0, attempt)? {
                        Classification::Decided(bounds) => bounds,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    source_bounds = Some(match source_bounds {
                        Some(existing) => match existing.union(&bounds, attempt) {
                            Classification::Decided(bounds) => bounds,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        },
                        None => bounds,
                    });
                    match source.direction {
                        BezierAlgebraicChordAxisDirection2::PositiveY
                        | BezierAlgebraicChordAxisDirection2::NegativeY => {
                            source_x_fibers.push(source.clone());
                        }
                        BezierAlgebraicChordAxisDirection2::PositiveX
                        | BezierAlgebraicChordAxisDirection2::NegativeX => {
                            source_y_fibers.push(source.clone());
                        }
                    }
                }
                let source_x_fibers = match sort_dedup_axis_aligned_algebraic_fibers(
                    source_x_fibers,
                    Axis2::X,
                    attempt,
                )? {
                    Classification::Decided(fibers) => fibers,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let source_y_fibers = match sort_dedup_axis_aligned_algebraic_fibers(
                    source_y_fibers,
                    Axis2::Y,
                    attempt,
                )? {
                    Classification::Decided(fibers) => fibers,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let Some(source_bounds) = source_bounds else {
                    return Err(CurveError::Topology(
                        "orthogonal erosion source did not contain a boundary span".into(),
                    ));
                };
                Ok(Classification::Decided(Arc::new(
                    ExactAxisAlignedAlgebraicErosionSource2 {
                        source_x_fibers: Arc::from(source_x_fibers),
                        source_y_fibers: Arc::from(source_y_fibers),
                        source_bounds,
                    },
                )))
            },
        );
        let source = match outcome.value? {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let _ = self
            .erosion_source
            .set((*policy, outcome.certainty, Arc::clone(&source)));
        Ok(Classification::Decided(source))
    }
}

fn exact_axis_aligned_algebraic_offset_loop(
    source_loop: Option<&ExactAxisAlignedAlgebraicSourceLoop2>,
    distance: &Real,
    corner_style: &OffsetCornerStyle2,
    component_contracts: bool,
    schedule_nonconvex_erosion: bool,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactAxisAlignedAlgebraicOffsetLoop2>> {
    let Some(source_loop) = source_loop else {
        return Ok(Classification::Decided(
            ExactAxisAlignedAlgebraicOffsetLoop2::Inapplicable,
        ));
    };
    let convex_inward =
        match axis_aligned_loop_is_convex_inward(&source_loop.spans, distance, policy) {
            Classification::Decided(convex_inward) => convex_inward,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if component_contracts && schedule_nonconvex_erosion && !convex_inward {
        let erosion_source = match source_loop.erosion_source(policy)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let erosion = exact_axis_aligned_algebraic_erosion(
            &erosion_source,
            &source_loop.spans,
            distance,
            fill_rule,
            policy,
        )?;
        return Ok(match erosion {
            Classification::Decided(boundaries) if boundaries.is_empty() => {
                Classification::Decided(ExactAxisAlignedAlgebraicOffsetLoop2::Removed)
            }
            Classification::Decided(boundaries) => Classification::Decided(
                ExactAxisAlignedAlgebraicOffsetLoop2::ErodedBoundaries(boundaries),
            ),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        });
    }
    let mut spans = Vec::with_capacity(source_loop.spans.len());
    for source in source_loop.spans.iter() {
        match exact_axis_aligned_algebraic_offset_span(
            &source.chord,
            source.direction,
            distance,
            policy,
        )? {
            Classification::Decided(span) => spans.push(span),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let mut joins = Vec::with_capacity(spans.len());
    for span_index in 0..spans.len() {
        let next_index = (span_index + 1) % spans.len();
        match exact_axis_aligned_algebraic_join(
            &spans[span_index],
            &spans[next_index],
            distance,
            corner_style,
            policy,
        )? {
            Classification::Decided(join) => joins.push(join),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    for (span_index, join) in joins.iter().enumerate() {
        let ExactAxisAlignedAlgebraicJoin2::Miter(miter) = join else {
            continue;
        };
        let next_index = (span_index + 1) % spans.len();
        spans[span_index].offset_end = miter.clone();
        spans[next_index].offset_start = miter.clone();
    }
    let has_reversed_span = match axis_aligned_offset_has_reversed_span(&spans, policy)? {
        Classification::Decided(has_reversed_span) => has_reversed_span,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if has_reversed_span && convex_inward {
        return Ok(Classification::Decided(
            ExactAxisAlignedAlgebraicOffsetLoop2::Removed,
        ));
    }
    if has_reversed_span {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let mut fragments = Vec::with_capacity(spans.len().saturating_mul(3));
    for span_index in 0..spans.len() {
        match append_exact_algebraic_line_join(
            &mut fragments,
            &spans[span_index].offset_start,
            &spans[span_index].offset_end,
            Some(spans[span_index].direction),
            [false; 2],
            policy,
        )? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let next_index = (span_index + 1) % spans.len();
        let join = match &joins[span_index] {
            ExactAxisAlignedAlgebraicJoin2::Shared | ExactAxisAlignedAlgebraicJoin2::Miter(_) => {
                Classification::Decided(())
            }
            ExactAxisAlignedAlgebraicJoin2::Line => append_exact_algebraic_line_join(
                &mut fragments,
                &spans[span_index].offset_end,
                &spans[next_index].offset_start,
                None,
                [false; 2],
                policy,
            )?,
            ExactAxisAlignedAlgebraicJoin2::Round(sweep_kind) => {
                append_exact_axis_aligned_algebraic_round_join(
                    &mut fragments,
                    &spans[span_index],
                    &spans[next_index],
                    distance,
                    *sweep_kind,
                    policy,
                )?
            }
        };
        match join {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    CurveRegionBoundaryLoop2::new(fragments, policy)
        .map(ExactAxisAlignedAlgebraicOffsetLoop2::Boundary)
        .map(Classification::Decided)
}

fn exact_rational_endpoint_evidence(
    curve: &RationalBezier2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierIntersectionPointEvidence2>> {
    Ok(
        match crate::rational_bezier_general::exact_contact_point_evidence(
            curve, parameter, policy,
        )? {
            Some(point) => Classification::Decided(point),
            None => Classification::Uncertain(UncertaintyReason::Unsupported),
        },
    )
}

fn exact_circular_algebraic_endpoint_tangent(
    curve: &RationalBezier2,
    parameter: &BezierParameter2,
    point: &RationalBezierIntersectionPointEvidence2,
    circle: &Arc<crate::rational_bezier::RationalQuadraticCircle2>,
    clockwise: bool,
    reversed: bool,
    policy: &CurveContext,
) -> Classification<Option<ExactOffsetTangent2>> {
    match parameter {
        BezierParameter2::Exact(parameter) => curve
            .derivative_at_classified(parameter, policy)
            .map(|derivative| {
                let tangent = (derivative.dx().clone(), derivative.dy().clone());
                Some(ExactOffsetTangent2::Vector(if reversed {
                    (-tangent.0, -tangent.1)
                } else {
                    tangent
                }))
            }),
        BezierParameter2::Algebraic(_) => {
            Classification::Decided(Some(ExactOffsetTangent2::CircularPoint {
                point: point.clone(),
                circle: Arc::clone(circle),
                clockwise: clockwise != reversed,
            }))
        }
    }
}

fn exact_offset_span_from_algebraic_endpoint_images(
    reversed: bool,
    start: &BezierParameter2,
    end: &BezierParameter2,
    source_curve: &Option<BezierSubcurve2>,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let Some(BezierSubcurve2::RationalQuadratic(source_curve)) = source_curve else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let source_arc = match crate::arc_bezier::rational_quadratic_circular_arc(source_curve, policy)?
    {
        Classification::Decided(Some(arc)) => arc,
        Classification::Decided(None) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let source_subcurve = BezierSubcurve2::RationalQuadratic(source_curve.clone());
    let source_rational = RationalBezier2::try_from_subcurve(&source_subcurve)?;
    let (traversal_start, traversal_end) = if reversed { (end, start) } else { (start, end) };
    let source_end =
        match exact_rational_endpoint_evidence(&source_rational, traversal_end, policy)? {
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
            let center =
                RationalBezierIntersectionPointEvidence2::Exact(source_arc.center().clone());
            return Ok(Classification::Decided(ExactOffsetSpan2 {
                fragments: Vec::new(),
                source_end,
                offset_start: center.clone(),
                offset_end: center,
                start_tangent: None,
                end_tangent: None,
            }));
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
            BezierParameter2::Exact(_) => Ok(None),
            BezierParameter2::Algebraic(parameter) => {
                Ok(Some(BezierAlgebraicEndpointImage2::from_source_curve(
                    &offset_subcurve,
                    parameter,
                    policy,
                )?))
            }
        }
    };
    let start_image = endpoint_image(start)?;
    let end_image = endpoint_image(end)?;
    let offset_rational = RationalBezier2::try_from_subcurve(&offset_subcurve)?;
    let offset_start =
        match exact_rational_endpoint_evidence(&offset_rational, traversal_start, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    let offset_end =
        match exact_rational_endpoint_evidence(&offset_rational, traversal_end, policy)? {
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
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments: vec![BezierSplitFragment2::AlgebraicEndpointImages {
            reversed,
            start: start.clone(),
            end: end.clone(),
            source_curve: Some(offset_subcurve),
            start_image,
            end_image,
        }],
        source_end,
        offset_start,
        offset_end,
        start_tangent,
        end_tangent,
    }))
}

fn exact_offset_span_from_materialized_curve(
    curve: &BezierSubcurve2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    if let Classification::Decided(segment) = materialized_native_subcurve_segment(curve, policy)? {
        let native_offset = match &segment {
            Segment2::Line(line) => {
                Classification::Decided(Segment2::Line(line.offset_left(distance.clone())?))
            }
            Segment2::Arc(arc) => {
                match exact_offset_span_from_native_arc(curve, arc, distance, policy)? {
                    decided @ Classification::Decided(_) => return Ok(decided),
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
        };
        if let Classification::Decided(offset) = native_offset {
            return exact_offset_span_from_native_segment(curve, &offset, policy);
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
    let analysis = match parallel.singularity_analysis(policy)? {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if !analysis.source_is_regular() {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
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
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments,
        source_end: curve.end().clone().into(),
        offset_start: offset_start.into(),
        offset_end: offset_end.into(),
        start_tangent: Some(ExactOffsetTangent2::Vector(start_tangent)),
        end_tangent: Some(ExactOffsetTangent2::Vector(end_tangent)),
    }))
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
            let center = RationalBezierIntersectionPointEvidence2::Exact(arc.center().clone());
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
    let direction = match chord.axis_direction(policy)? {
        Classification::Decided(direction) => direction,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if let Some(direction) = direction {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-span",
            "axis-algebraic-chord",
        );
        let offset =
            match exact_axis_aligned_algebraic_offset_span(chord, direction, distance, policy)? {
                Classification::Decided(offset) => offset,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let offset_chord = crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
            offset.offset_start.clone(),
            offset.offset_end.clone(),
            direction,
            policy,
        );
        let tangent = direction.unit_tangent();
        return Ok(Classification::Decided(ExactOffsetSpan2 {
            fragments: vec![retained_chord_or_exact_line_fragment(offset_chord)?],
            source_end: chord.end().clone(),
            offset_start: offset.offset_start,
            offset_end: offset.offset_end,
            start_tangent: Some(ExactOffsetTangent2::Vector(tangent.clone())),
            end_tangent: Some(ExactOffsetTangent2::Vector(tangent)),
        }));
    }

    let Some(tangent) = chord.certified_unit_tangent() else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-span",
            "retained-oblique-algebraic-chord",
        );
        let offset_chord = chord.parallel_left_retained(distance.clone(), policy)?;
        let offset_start = offset_chord.start().clone();
        let offset_end = offset_chord.end().clone();
        return Ok(Classification::Decided(ExactOffsetSpan2 {
            fragments: vec![retained_chord_or_exact_line_fragment(offset_chord)?],
            source_end: chord.end().clone(),
            offset_start,
            offset_end,
            start_tangent: Some(ExactOffsetTangent2::AlgebraicChord(chord.clone())),
            end_tangent: Some(ExactOffsetTangent2::AlgebraicChord(chord.clone())),
        }));
    };
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-span",
        "certified-oblique-algebraic-chord",
    );
    let normal_offset = (-(&tangent.1 * distance), &tangent.0 * distance);
    let offset_chord = match chord.translated(&normal_offset.0, &normal_offset.1, policy)? {
        Classification::Decided(chord) => chord,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let offset_start = offset_chord.start().clone();
    let offset_end = offset_chord.end().clone();
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments: vec![retained_chord_or_exact_line_fragment(offset_chord)?],
        source_end: chord.end().clone(),
        offset_start,
        offset_end,
        start_tangent: Some(ExactOffsetTangent2::Vector(tangent.clone())),
        end_tangent: Some(ExactOffsetTangent2::Vector(tangent)),
    }))
}

fn exact_algebraic_cusp_semicircle_endpoint(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    at_start: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierIntersectionPointEvidence2>> {
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
    source_endpoint: &RationalBezierIntersectionPointEvidence2,
    at_start: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierIntersectionPointEvidence2>> {
    match source.translated_cardinal_offset_endpoint(offset, at_start, source_endpoint, policy)? {
        Classification::Decided(Some(point)) => Ok(Classification::Decided(point)),
        Classification::Decided(None) => {
            match source.concentric_offset_endpoint_point_evidence(offset, at_start, policy)? {
                Classification::Decided(Some(point)) => Ok(Classification::Decided(point)),
                Classification::Decided(None) => {
                    exact_algebraic_cusp_semicircle_endpoint(offset, at_start, policy)
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn exact_offset_algebraic_cusp_semicircle_tangent(
    offset: &crate::BezierAlgebraicCuspSemicircleFragment2,
    at_start: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<ExactOffsetTangent2>>> {
    match offset.endpoint_chord_tangent_cross(at_start, policy)? {
        Classification::Decided(Some((chord, circle_cross_chord))) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent",
                "selected-circle-chord-contact",
            );
            return Ok(Classification::Decided(Some(
                ExactOffsetTangent2::ChordContact {
                    fragment: offset.clone(),
                    at_start,
                    chord,
                    circle_cross_chord,
                },
            )));
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
                || ExactOffsetTangent2::SelectedCircularEndpoint {
                    fragment: offset.clone(),
                    at_start,
                },
                ExactOffsetTangent2::Vector,
            ))
        });
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-tangent",
        match &tangent {
            Classification::Decided(Some(ExactOffsetTangent2::SelectedCircularEndpoint {
                ..
            })) => "retained-selected-circle-endpoint",
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
            return Ok(Classification::Uncertain(reason));
        }
    };
    let source_end = match exact_algebraic_cusp_semicircle_endpoint(fragment, false, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
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
            let center = RationalBezierIntersectionPointEvidence2::Algebraic(
                fragment.semicircle().center_point_image(policy)?,
            );
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
            return Ok(Classification::Uncertain(reason));
        }
    };
    let start_tangent =
        match exact_offset_algebraic_cusp_semicircle_tangent(&offset_fragment, true, policy)? {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    let end_tangent =
        match exact_offset_algebraic_cusp_semicircle_tangent(&offset_fragment, false, policy)? {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => {
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

fn exact_offset_span_from_analytic_parallel(
    fragment: &crate::BezierParallelFragment2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let Some((range_start, range_end)) = fragment.range().exact_endpoints() else {
        // The composed carrier itself remains exact at algebraic parameters,
        // but a distinct adjacent carrier can require a join whose center and
        // endpoints live in a not-yet-supported algebraic coordinate tower.
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    };
    let source_scale = match fragment
        .parallel()
        .regular_fragment_derivative_scale_sign(fragment.range(), policy)?
    {
        Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let traversal_agrees_with_source =
        (source_scale == RealSign::Positive) != fragment.is_reversed();
    let composed_distance = if traversal_agrees_with_source {
        fragment.parallel().distance() + distance
    } else {
        fragment.parallel().distance() - distance
    };
    let composed = fragment.parallel().with_distance(composed_distance);
    let composed_distance_sign = match real_sign(composed.distance(), policy) {
        Some(sign) => sign,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };

    let mut boundaries = Vec::new();
    boundaries.push(fragment.range().start().clone());
    if composed_distance_sign != RealSign::Zero {
        let analysis = match composed.singularity_analysis(policy)? {
            Classification::Decided(analysis) => analysis,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for singularity in analysis.source_singularities() {
            let after_start =
                match singularity.cmp_by_refinement(fragment.range().start(), policy)? {
                    Classification::Decided(order) => !order.is_lt(),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let before_end = match singularity.cmp_by_refinement(fragment.range().end(), policy)? {
                Classification::Decided(order) => !order.is_gt(),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if after_start && before_end {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        }
        for cusp in analysis.parallel_cusps() {
            let after_start = match cusp.cmp_by_refinement(fragment.range().start(), policy)? {
                Classification::Decided(order) => order.is_gt(),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let before_end = match cusp.cmp_by_refinement(fragment.range().end(), policy)? {
                Classification::Decided(order) => order.is_lt(),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if after_start && before_end {
                let order = cusp.cmp_by_refinement(
                    boundaries
                        .last()
                        .expect("composed parallel boundaries begin at the retained range start"),
                    policy,
                )?;
                match order {
                    Classification::Decided(std::cmp::Ordering::Greater) => {
                        boundaries.push(cusp.clone());
                    }
                    Classification::Decided(std::cmp::Ordering::Equal) => {}
                    Classification::Decided(std::cmp::Ordering::Less) => {
                        return Err(CurveError::Topology(
                            "composed parallel cusp isolators are not ordered".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
    }
    boundaries.push(fragment.range().end().clone());
    let ranges = boundaries
        .windows(2)
        .map(|pair| BezierParameterRange2::new_validated(pair[0].clone(), pair[1].clone()))
        .collect::<Vec<_>>();
    let fragments = exact_parallel_fragments(&composed, &boundaries, fragment.is_reversed());
    let first_range = ranges
        .first()
        .expect("a retained parallel range produces at least one composed range");
    let last_range = ranges
        .last()
        .expect("a retained parallel range produces at least one composed range");
    let (start_parameter, end_parameter, start_range, end_range) = if fragment.is_reversed() {
        (range_end, range_start, last_range, first_range)
    } else {
        (range_start, range_end, first_range, last_range)
    };
    let source_end = match fragment.parallel().point_at(end_parameter, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let offset_start = match composed.point_at(start_parameter, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let offset_end = match composed.point_at(end_parameter, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let start_scale = match composed.regular_fragment_derivative_scale_sign(start_range, policy)? {
        Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_scale = if ranges.len() == 1 {
        start_scale
    } else {
        match composed.regular_fragment_derivative_scale_sign(end_range, policy)? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    };
    let start_tangent = match exact_parallel_limiting_tangent(
        &composed,
        start_parameter,
        start_scale,
        fragment.is_reversed(),
        policy,
    )? {
        Classification::Decided(tangent) => tangent,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_tangent = match exact_parallel_limiting_tangent(
        &composed,
        end_parameter,
        end_scale,
        fragment.is_reversed(),
        policy,
    )? {
        Classification::Decided(tangent) => tangent,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments,
        source_end: source_end.into(),
        offset_start: offset_start.into(),
        offset_end: offset_end.into(),
        start_tangent: Some(ExactOffsetTangent2::Vector(start_tangent)),
        end_tangent: Some(ExactOffsetTangent2::Vector(end_tangent)),
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
            | BezierSplitFragment2::AlgebraicEndpointImages { .. }
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::Unresolved { .. } => None,
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

    fn range(self) -> CurveRegionParameterRange2 {
        match self {
            Self::Analytic(fragment) => {
                CurveRegionParameterRange2::from_bezier_range(fragment.range().clone())
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

fn retained_parallel_represented_parameter(parameter: &CurveRegionParameter2) -> Option<&Real> {
    parameter.as_exact().or_else(|| {
        parameter
            .as_selected_fiber()
            .and_then(|parameter| parameter.represented_value())
    })
}

fn retained_parallel_traversal_start(
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
) -> CurveRegionParameter2 {
    let range = fragment.range();
    if fragment.is_reversed() {
        range.end().clone()
    } else {
        range.start().clone()
    }
}

fn retained_parallel_traversal_end(
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
) -> CurveRegionParameter2 {
    let range = fragment.range();
    if fragment.is_reversed() {
        range.start().clone()
    } else {
        range.end().clone()
    }
}

fn exact_retained_parallel_fragment(
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
    parallel: BezierParallel2,
) -> Option<crate::BezierParallelFragment2> {
    let range = fragment.range();
    let start = retained_parallel_represented_parameter(range.start())?.clone();
    let end = retained_parallel_represented_parameter(range.end())?.clone();
    Some(crate::BezierParallelFragment2::from_certified_range(
        parallel,
        BezierParameterRange2::new_validated(
            BezierParameter2::Exact(start),
            BezierParameter2::Exact(end),
        ),
        fragment.is_reversed(),
    ))
}

fn retained_parallel_fragment_scale_sign(
    parallel: &BezierParallel2,
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let range = fragment.range();
    let parameter = match range
        .start()
        .strict_rational_between_ordered(range.end(), policy)?
    {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    parallel.parallel_derivative_scale_sign(&BezierParameter2::Exact(parameter), policy)
}

fn exact_offset_span_from_retained_parallel_fragment(
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let parallel = fragment.parallel();
    let Some(fragment) = exact_retained_parallel_fragment(fragment, parallel) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    };
    exact_offset_span_from_analytic_parallel(&fragment, distance, policy)
}

/// Coalesces one traversal-contiguous retained-parallel run whose only
/// non-represented boundaries are internal arrangement partitions.
///
/// Analytic-parallel and selected-fiber fragments share this authority. A
/// Boolean or self-contact split does not create a geometric corner, so a run
/// with one carrier, traversal, and derivative-scale sign is recovered between
/// represented outer endpoints and composed once. A scale-sign change is an
/// old cusp and deliberately remains a real span boundary.
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
    if exact_retained_parallel_fragment(first, parallel.clone()).is_some()
        || retained_parallel_represented_parameter(&retained_parallel_traversal_start(first))
            .is_none()
    {
        return Ok(Classification::Decided(None));
    }
    let scale = match retained_parallel_fragment_scale_sign(&parallel, first, policy)? {
        Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

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
        let next_scale = match retained_parallel_fragment_scale_sign(&parallel, next, policy)? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if next_scale != scale {
            break;
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

fn exact_parallel_limiting_tangent(
    parallel: &BezierParallel2,
    parameter: &Real,
    scale: RealSign,
    reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<(Real, Real)>> {
    debug_assert_ne!(scale, RealSign::Zero);
    let tangent = match parallel.source_tangent_at(parameter, policy)? {
        Classification::Decided(tangent) => tangent,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(
        if (scale == RealSign::Positive) != reversed {
            tangent
        } else {
            (-tangent.0, -tangent.1)
        },
    ))
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
        start_tangent: Some(ExactOffsetTangent2::Vector(start_tangent)),
        end_tangent: Some(ExactOffsetTangent2::Vector(end_tangent)),
    }))
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
    let endpoint_equality = previous.offset_end.same_point(&next.offset_start, policy);
    match endpoint_equality {
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
    let (Some(previous_tangent), Some(next_tangent)) = (&previous.end_tangent, &next.start_tangent)
    else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-join",
            "missing-tangent",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let turn_sign = match exact_offset_tangent_cross_sign(previous_tangent, next_tangent, policy) {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let distance_sign = match real_sign(distance, policy) {
        Some(sign) => sign,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    let inward = exact_sign_product(turn_sign, distance_sign) == RealSign::Positive;
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
            let opposite =
                match exact_offset_tangents_are_opposite(previous_tangent, next_tangent, policy) {
                    Classification::Decided(opposite) => opposite,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
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
        OffsetCornerStyle2::Miter { limit } if !inward => {
            append_exact_miter_join(fragments, previous, next, distance, Some(limit), policy)
        }
        OffsetCornerStyle2::Round
        | OffsetCornerStyle2::Bevel
        | OffsetCornerStyle2::Miter { .. } => {
            append_exact_miter_join(fragments, previous, next, distance, None, policy)
        }
    }
}

fn exact_offset_parallel_tangent_contact(
    span: &ExactOffsetSpan2,
    at_start: bool,
    point: &Point2,
) -> Option<crate::rational_bezier::RationalQuadraticParallelCircleContact2> {
    let fragment = if at_start {
        span.fragments.first()
    } else {
        span.fragments.last()
    };
    let BezierSplitFragment2::AnalyticParallel(fragment) = fragment? else {
        return None;
    };
    let parameter = if at_start {
        analytic_parallel_traversal_start(fragment)
    } else {
        analytic_parallel_traversal_end(fragment)
    }
    .as_exact()?
    .clone();
    Some(
        crate::rational_bezier::RationalQuadraticParallelCircleContact2 {
            parallel: fragment.parallel().clone(),
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
    let fragment = if at_start {
        span.fragments.first()
    } else {
        span.fragments.last()
    };
    let BezierSplitFragment2::AnalyticParallel(fragment) = fragment? else {
        return None;
    };
    let parameter = if at_start {
        analytic_parallel_traversal_start(fragment)
    } else {
        analytic_parallel_traversal_end(fragment)
    }
    .as_exact()?
    .clone();
    Some(BezierParallelLineTangentContact2::new(
        fragment.parallel().clone(),
        parameter,
        line_endpoint,
        fragment.is_reversed(),
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
    let (Some(previous_offset_end), Some(next_offset_start), Some(center)) = (
        previous.offset_end.as_exact(),
        next.offset_start.as_exact(),
        previous.source_end.as_exact(),
    ) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
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
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let (
        Some(ExactOffsetTangent2::Vector(previous_tangent)),
        Some(ExactOffsetTangent2::Vector(next_tangent)),
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
        previous.offset_end.as_exact(),
        next.offset_start.as_exact(),
        previous.source_end.as_exact(),
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
    let delta = next_offset_start.delta_from(previous_offset_end);
    let numerator = &delta.0 * &next_tangent.1 - &delta.1 * &next_tangent.0;
    let parameter = (numerator / denominator)?;
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
    point: &RationalBezierIntersectionPointEvidence2,
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

fn exact_offset_tangent_cross_sign(
    first: &ExactOffsetTangent2,
    second: &ExactOffsetTangent2,
    policy: &CurveContext,
) -> Classification<RealSign> {
    match (first, second) {
        (ExactOffsetTangent2::Vector(first), ExactOffsetTangent2::Vector(second)) => {
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
            ExactOffsetTangent2::AlgebraicChord(first),
            ExactOffsetTangent2::AlgebraicChord(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "algebraic-chord-algebraic-chord",
            );
            match first.tangent_cross_sign(second, policy) {
                Ok(sign) => sign,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (ExactOffsetTangent2::AlgebraicChord(first), ExactOffsetTangent2::Vector(second)) => {
            match first.tangent_cross_vector_sign(second, policy) {
                Ok(sign) => sign,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (ExactOffsetTangent2::Vector(first), ExactOffsetTangent2::AlgebraicChord(second)) => {
            match second.tangent_cross_vector_sign(first, policy) {
                Ok(sign) => sign.map(exact_sign_reverse),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            ExactOffsetTangent2::CircularPoint {
                point,
                circle,
                clockwise,
            },
            ExactOffsetTangent2::Vector(second),
        ) => exact_circular_tangent_cross_vector(point, circle, *clockwise, second, policy),
        (
            ExactOffsetTangent2::Vector(first),
            ExactOffsetTangent2::CircularPoint {
                point,
                circle,
                clockwise,
            },
        ) => exact_circular_tangent_cross_vector(point, circle, *clockwise, first, policy)
            .map(exact_sign_reverse),
        (
            ExactOffsetTangent2::SelectedCircularEndpoint { fragment, at_start },
            ExactOffsetTangent2::Vector(second),
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
            ExactOffsetTangent2::Vector(first),
            ExactOffsetTangent2::SelectedCircularEndpoint { fragment, at_start },
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
            ExactOffsetTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
            },
            ExactOffsetTangent2::Vector(second),
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
            ExactOffsetTangent2::Vector(first),
            ExactOffsetTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
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
            ExactOffsetTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
            },
            ExactOffsetTangent2::AlgebraicChord(second),
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
            ExactOffsetTangent2::AlgebraicChord(first),
            ExactOffsetTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
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
            ExactOffsetTangent2::SelectedCircularEndpoint { fragment, at_start },
            ExactOffsetTangent2::AlgebraicChord(second),
        ) => fragment
            .endpoint_tangent_cross_algebraic_chord(*at_start, second, true, policy)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
        (
            ExactOffsetTangent2::AlgebraicChord(first),
            ExactOffsetTangent2::SelectedCircularEndpoint { fragment, at_start },
        ) => fragment
            .endpoint_tangent_cross_algebraic_chord(*at_start, first, true, policy)
            .map(|cross| cross.map(exact_sign_reverse))
            .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
        (
            ExactOffsetTangent2::SelectedCircularEndpoint {
                fragment: first_fragment,
                at_start: first_start,
            },
            ExactOffsetTangent2::SelectedCircularEndpoint {
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
            match first_fragment.endpoint_pair_tangent_cross(
                *first_start,
                second_fragment,
                *second_start,
                policy,
            ) {
                Ok(Classification::Decided(Some(sign))) => Classification::Decided(sign),
                Ok(Classification::Decided(None)) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (ExactOffsetTangent2::ChordContact { .. }, ExactOffsetTangent2::ChordContact { .. }) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        (ExactOffsetTangent2::ChordContact { .. }, ExactOffsetTangent2::CircularPoint { .. })
        | (ExactOffsetTangent2::CircularPoint { .. }, ExactOffsetTangent2::ChordContact { .. })
        | (ExactOffsetTangent2::CircularPoint { .. }, ExactOffsetTangent2::CircularPoint { .. })
        | (
            ExactOffsetTangent2::SelectedCircularEndpoint { .. },
            ExactOffsetTangent2::CircularPoint { .. } | ExactOffsetTangent2::ChordContact { .. },
        )
        | (
            ExactOffsetTangent2::CircularPoint { .. } | ExactOffsetTangent2::ChordContact { .. },
            ExactOffsetTangent2::SelectedCircularEndpoint { .. },
        )
        | (ExactOffsetTangent2::AlgebraicChord(_), ExactOffsetTangent2::CircularPoint { .. })
        | (ExactOffsetTangent2::CircularPoint { .. }, ExactOffsetTangent2::AlgebraicChord(_)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
    }
}

fn exact_offset_tangents_are_opposite(
    first: &ExactOffsetTangent2,
    second: &ExactOffsetTangent2,
    policy: &CurveContext,
) -> Classification<bool> {
    match exact_offset_tangent_cross_sign(first, second, policy) {
        Classification::Decided(RealSign::Negative | RealSign::Positive) => {
            return Classification::Decided(false);
        }
        Classification::Decided(RealSign::Zero) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    match (first, second) {
        (ExactOffsetTangent2::Vector(first), ExactOffsetTangent2::Vector(second)) => {
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
            ExactOffsetTangent2::AlgebraicChord(first),
            ExactOffsetTangent2::AlgebraicChord(second),
        ) => match first.tangent_dot_sign(second, policy) {
            Ok(Classification::Decided(RealSign::Negative)) => Classification::Decided(true),
            Ok(Classification::Decided(RealSign::Positive)) => Classification::Decided(false),
            Ok(Classification::Decided(RealSign::Zero)) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
            Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
        },
        (ExactOffsetTangent2::AlgebraicChord(chord), ExactOffsetTangent2::Vector(vector))
        | (ExactOffsetTangent2::Vector(vector), ExactOffsetTangent2::AlgebraicChord(chord)) => {
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
            ExactOffsetTangent2::SelectedCircularEndpoint { fragment, at_start },
            ExactOffsetTangent2::Vector(vector),
        )
        | (
            ExactOffsetTangent2::Vector(vector),
            ExactOffsetTangent2::SelectedCircularEndpoint { fragment, at_start },
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
        (ExactOffsetTangent2::CircularPoint { .. }, _)
        | (_, ExactOffsetTangent2::CircularPoint { .. }) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        (ExactOffsetTangent2::SelectedCircularEndpoint { .. }, _)
        | (_, ExactOffsetTangent2::SelectedCircularEndpoint { .. }) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        (ExactOffsetTangent2::ChordContact { .. }, ExactOffsetTangent2::Vector(_))
        | (ExactOffsetTangent2::Vector(_), ExactOffsetTangent2::ChordContact { .. })
        | (ExactOffsetTangent2::ChordContact { .. }, ExactOffsetTangent2::ChordContact { .. })
        | (ExactOffsetTangent2::ChordContact { .. }, ExactOffsetTangent2::AlgebraicChord(_))
        | (ExactOffsetTangent2::AlgebraicChord(_), ExactOffsetTangent2::ChordContact { .. }) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
    }
}

fn offset_vectors_are_structurally_opposite(first: &(Real, Real), second: &(Real, Real)) -> bool {
    (&first.0 + &second.0).zero_status() == hyperreal::ZeroKnowledge::Zero
        && (&first.1 + &second.1).zero_status() == hyperreal::ZeroKnowledge::Zero
}

const fn curve_region_role_depth(role: CurveRegionLoopRole) -> i32 {
    match role {
        CurveRegionLoopRole::Material => 1,
        CurveRegionLoopRole::Hole => -1,
    }
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
            data.signed_loop_composition = self.data.signed_loop_composition;
            data.certified_regularized_filled_left_topology =
                self.data.certified_regularized_filled_left_topology;
            self.data = Arc::new(data);
        }
        Arc::get_mut(&mut self.data).expect("CurveRegion2 construction data is uniquely owned")
    }

    /// Constructs an empty unified region.
    pub fn empty() -> Self {
        Self::default()
    }

    /// Arranges unordered exact line/arc segments into unified region topology.
    ///
    /// The specialized native arrangement remains the retained fast path and
    /// diagnostic source, while any materialized output is promoted immediately
    /// into `CurveRegion2`.
    pub fn arrange_unordered_segments(
        source_segments: Vec<Segment2>,
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveRegionArrangement2>> {
        resolve_certified_operation(policy, |attempt| {
            let arrangement =
                LineArcRegion2::arrange_unordered_segments(source_segments, fill_rule, attempt)
                    .map_err(curve_region_promotion_error)?;
            promote_native_region_arrangement(arrangement, attempt)
        })
    }

    /// Arranges borrowed unordered exact line/arc segments into unified topology.
    pub fn arrange_unordered_segments_borrowed(
        source_segments: &[Segment2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveRegionArrangement2>> {
        resolve_certified_operation(policy, |attempt| {
            let arrangement = LineArcRegion2::arrange_unordered_segments_borrowed(
                source_segments,
                fill_rule,
                attempt,
            )
            .map_err(curve_region_promotion_error)?;
            promote_native_region_arrangement(arrangement, attempt)
        })
    }

    /// Arranges unordered exact lines through the specialized line pipeline.
    pub fn arrange_unordered_line_segments(
        source_segments: Vec<LineSeg2>,
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveRegionArrangement2>> {
        resolve_certified_operation(policy, |attempt| {
            let arrangement = LineArcRegion2::arrange_unordered_line_segments(
                source_segments,
                fill_rule,
                attempt,
            )
            .map_err(curve_region_promotion_error)?;
            promote_native_region_arrangement(arrangement, attempt)
        })
    }

    /// Arranges borrowed unordered exact lines through the specialized line pipeline.
    pub fn arrange_unordered_line_segments_borrowed(
        source_segments: &[LineSeg2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveRegionArrangement2>> {
        resolve_certified_operation(policy, |attempt| {
            let arrangement = LineArcRegion2::arrange_unordered_line_segments_borrowed(
                source_segments,
                fill_rule,
                attempt,
            )
            .map_err(curve_region_promotion_error)?;
            promote_native_region_arrangement(arrangement, attempt)
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
            Self::try_from_native_contours_raw(material_contours, hole_contours, attempt)
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
            None,
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

    /// Nests unordered native boundary contours and promotes their decided roles.
    ///
    /// Even containment depth becomes material and odd depth becomes a hole.
    /// Intersecting, touching, or otherwise uncertifiable boundaries remain an
    /// explicit uncertainty.
    pub fn try_from_native_boundary_contours(
        contours: Vec<Contour2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<Self>>> {
        resolve_certified_operation(
            policy,
            |attempt| match LineArcRegion2::from_boundary_contours(contours, attempt)
                .map_err(curve_region_promotion_error)?
            {
                Classification::Decided(region) => {
                    Self::try_from_line_arc_region_raw(&region, attempt)
                        .map(Classification::Decided)
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            },
        )
    }

    /// Borrowed counterpart to [`Self::try_from_native_boundary_contours`].
    pub fn try_from_native_boundary_contours_borrowed(
        contours: &[Contour2],
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<Self>>> {
        Self::try_from_native_boundary_contours(contours.to_vec(), policy)
    }

    /// Constructs unified filled topology from one possibly self-intersecting
    /// native line/arc contour.
    ///
    /// Certified self-intersection points split the authored traversal into
    /// simple cycles. [`FillRule::EvenOdd`] cycles are XORed, while
    /// [`FillRule::NonZero`] cycles are accumulated as exact integer winding
    /// layers. Positive-dimensional overlap fragments are paired by geometric
    /// image rather than construction provenance. The specialized native
    /// topology remains private implementation state; callers receive the
    /// authoritative unified carrier.
    pub fn try_from_regularized_native_contour(
        contour: &Contour2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            regularize_native_contour_with_curve_region(contour, attempt)
                .map(Classification::Decided)
        })
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
    /// must be supplied for every boundary path. The semantics remain attached
    /// across exact affine transforms and regenerated native fast paths.
    pub fn try_from_boundary_paths_with_loop_semantics(
        paths: &[CurvePath2],
        roles: &[CurveRegionLoopRole],
        fill_rules: &[FillRule],
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            Self::try_from_boundary_paths_with_loop_semantics_raw(
                paths, roles, fill_rules, attempt, None,
            )
        })
    }

    /// Constructs an exact signed composition from independently authored loops.
    ///
    /// Unlike a regularized region, material and hole operands may cross or
    /// overlap. Point queries therefore accumulate each loop's certified signed
    /// contribution directly instead of using a native region accelerator that
    /// assumes disjoint material/hole topology. Native contours remain retained
    /// for exact manufacturing output.
    pub fn try_from_signed_boundary_paths_with_loop_semantics(
        paths: &[CurvePath2],
        roles: &[CurveRegionLoopRole],
        fill_rules: &[FillRule],
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            let mut region = Self::try_from_boundary_paths_with_loop_semantics_raw(
                paths, roles, fill_rules, attempt, None,
            )?;
            region.data_mut_for_construction().signed_loop_composition = true;
            Ok(region)
        })
    }

    /// Constructs a curved region with explicit loop roles, fill rules, and
    /// authored interior sides.
    ///
    /// This is the immediate exact constructor for carriers whose signed-area
    /// integral is not yet representable, including nonuniform general
    /// rational Beziers. The supplied side is topology evidence, not an
    /// approximation: `Left` states that filled material lies to the left
    /// while traversing the corresponding path, and `Right` states the
    /// opposite. One entry must be supplied for every path.
    pub fn try_from_boundary_paths_with_loop_topology(
        paths: &[CurvePath2],
        roles: &[CurveRegionLoopRole],
        fill_rules: &[FillRule],
        interior_sides: &[CurveBoundaryInteriorSide2],
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            if paths.len() != interior_sides.len() {
                let family = paths
                    .first()
                    .map_or(CurveFamily2::Line, |path| path.curves()[0].family());
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Construction,
                    family,
                    CurveError::Topology(
                        "curved-region interior sides must match boundary path count".into(),
                    ),
                ));
            }
            Self::try_from_boundary_paths_with_loop_semantics_raw(
                paths,
                roles,
                fill_rules,
                attempt,
                Some(
                    interior_sides
                        .iter()
                        .map(|side| *side == CurveBoundaryInteriorSide2::Left)
                        .collect(),
                ),
            )
        })
    }

    pub(crate) fn try_from_boundary_paths_with_loop_semantics_raw(
        paths: &[CurvePath2],
        roles: &[CurveRegionLoopRole],
        fill_rules: &[FillRule],
        policy: &CurveContext,
        certified_filled_sides: Option<Vec<bool>>,
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
        if let Some(filled_sides) = certified_filled_sides {
            region = region
                .with_certified_filled_side_is_left(filled_sides)
                .map_err(curve_region_promotion_error)?;
        }
        if let Some(native) = native_region_from_curve_paths(paths, roles, fill_rules)
            .map_err(curve_region_promotion_error)?
        {
            region.data.line_image_region.certify(Some(native));
        }
        Ok(region)
    }

    /// Constructs a top-level exact curved region from closed boundary paths.
    ///
    /// Every authored family is promoted through its clone-shared native
    /// topology once.
    pub fn try_from_boundary_paths(
        paths: &[CurvePath2],
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            Self::try_from_boundary_paths_raw(paths, attempt)
        })
    }

    fn try_from_boundary_paths_raw(
        paths: &[CurvePath2],
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let mut boundary_loops = Vec::with_capacity(paths.len());
        let mut next_arrangement_fragment_index = 0;
        for path in paths {
            match crate::curve::validate_closed_curve_path_connectivity(path, policy)
                .map_err(|error| error.with_operation(CurveOperation2::Construction))?
            {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Construction,
                        path.curves()[0].family(),
                        reason,
                    ));
                }
            }
            let fragment_capacity = match path.native_bezier_fragments_with_policy(policy)? {
                Classification::Decided(fragments) => fragments.len(),
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Construction,
                        path.curves()[0].family(),
                        reason,
                    ));
                }
            };
            let mut fragments = Vec::with_capacity(fragment_capacity);
            let mut arrangement_sources = Vec::with_capacity(fragment_capacity);
            for curve in path.curves() {
                let native_fragments = match curve.native_bezier_fragments_with_policy(policy)? {
                    Classification::Decided(fragments) => fragments,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Construction,
                            curve.family(),
                            reason,
                        ));
                    }
                };
                for native in native_fragments {
                    let arrangement_fragment_index = next_arrangement_fragment_index;
                    next_arrangement_fragment_index += 1;
                    fragments.push(BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: native.curve().clone(),
                    });
                    arrangement_sources.push(CurveRegionFragmentSource2::new(
                        arrangement_fragment_index,
                        arrangement_fragment_index,
                        0,
                    ));
                }
            }
            let boundary_loop = CurveRegionBoundaryLoop2::try_new_with_arrangement_sources(
                fragments,
                arrangement_sources,
                policy,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Construction,
                    path.curves()[0].family(),
                    cause,
                )
            })?;
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
        let similarity = std::cell::OnceCell::new();
        let mut semicircle_similarity_cache =
            BezierAlgebraicCuspSemicircleSimilarityCache2::default();
        for boundary in &self.data.boundary_loops {
            let fragments = boundary
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
            let boundary = match boundary.arrangement_sources() {
                Some(sources) => {
                    CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                        fragments,
                        sources.to_vec(),
                        policy,
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
            data.signed_loop_composition = self.data.signed_loop_composition;
            data.certified_regularized_filled_left_topology =
                self.data.certified_regularized_filled_left_topology;
        }
        let sides = match self
            .filled_side_is_left_raw(policy)
            .map_err(affine_region_error)?
        {
            Classification::Decided(sides) => sides
                .iter()
                .map(|side| if orientation_reversing { !side } else { *side })
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

    /// Constructs an exact curved region from already materialized boundary loops.
    pub fn new(boundary_loops: Vec<CurveRegionBoundaryLoop2>) -> CurveResult<Self> {
        if boundary_loops.is_empty() {
            return Ok(Self::default());
        }
        validate_retained_region_loops(&boundary_loops)?;
        Ok(Self::from_certified_boundary_loops(boundary_loops))
    }

    /// Constructs retained exact loops with explicit role, fill, and interior-side topology.
    ///
    /// This is the authoritative constructor for procedural carriers whose
    /// Green integral is not represented as a native [`Real`], including
    /// analytic Bezier parallels. The interior side is authored topology
    /// evidence; it is never inferred from a finite projection.
    pub fn try_new_with_loop_topology(
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

    /// Publishes independently certified regularized filled-left topology.
    ///
    /// An authoritative filled-left face walk is the general producer. Narrow
    /// exact geometric proofs, such as the one-turn cardinal convex parallel
    /// certificate, may publish the same fact. Unlike authored boundary
    /// provenance, this marker certifies that every retained chain is a
    /// noncrossing regularized boundary. Expensive exact nesting may therefore
    /// be deferred until a caller actually needs loop roles.
    pub(crate) fn with_certified_regularized_filled_left_topology(mut self) -> CurveResult<Self> {
        if self.data.boundary_loops.iter().any(|boundary_loop| {
            !boundary_loop.has_arrangement_sources() || boundary_loop.is_empty()
        }) {
            return Err(CurveError::Topology(
                "regularized filled-left topology requires arrangement provenance".into(),
            ));
        }
        let loop_count = self.data.boundary_loops.len();
        let data = self.data_mut_for_construction();
        data.filled_side_is_left
            .certify(Arc::from(vec![true; loop_count]));
        data.certified_regularized_filled_left_topology = true;
        Ok(self)
    }

    pub(crate) fn has_certified_regularized_filled_left_topology(&self) -> bool {
        self.data.certified_regularized_filled_left_topology
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

        match self.curved_nesting_role_evidence_raw(policy)? {
            Classification::Decided(evidence) => {
                return filled_sides_from_roles_and_areas(
                    evidence.roles(),
                    evidence.signed_areas(),
                    policy,
                )
                .map(|sides| Classification::Decided(Arc::from(sides)));
            }
            Classification::Uncertain(UncertaintyReason::Unsupported) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        match self.line_image_role_evidence_raw(policy)? {
            Classification::Decided(evidence) => {
                let mut areas = Vec::with_capacity(evidence.contours().len());
                for contour in evidence.contours() {
                    let Some(area) = contour.signed_area()? else {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    };
                    areas.push(area);
                }
                filled_sides_from_roles_and_areas(evidence.roles(), &areas, policy)
                    .map(|sides| Classification::Decided(Arc::from(sides)))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Materializes retained region carriers from a decided retained traversal.
    ///
    /// Every traversal chain must be closed. Materialized native fragments and
    /// algebraic endpoint-image fragments are accepted as exact carriers;
    /// unresolved fragments remain explicit boundary uncertainty. This mirrors
    /// Materialized native fragments and retained algebraic carriers enter the
    /// same authoritative region representation.
    pub fn from_retained_arrangement_traversal(
        graph: &BezierArrangementGraph2,
        traversal: &BezierArrangementTraversal2,
        policy: &CurveContext,
    ) -> CurveOutcome<Classification<Self>> {
        resolve_certified_value(policy, |attempt| {
            Self::from_retained_arrangement_traversal_raw(graph, traversal, attempt)
        })
    }

    pub(crate) fn from_retained_arrangement_traversal_raw(
        graph: &BezierArrangementGraph2,
        traversal: &BezierArrangementTraversal2,
        policy: &CurveContext,
    ) -> Classification<Self> {
        Self::from_retained_arrangement_traversal_impl(graph, traversal, Some(policy))
    }

    pub(crate) fn from_certified_retained_arrangement_traversal(
        graph: &BezierArrangementGraph2,
        traversal: &BezierArrangementTraversal2,
    ) -> Classification<Self> {
        Self::from_retained_arrangement_traversal_impl(graph, traversal, None)
    }

    fn from_retained_arrangement_traversal_impl(
        graph: &BezierArrangementGraph2,
        traversal: &BezierArrangementTraversal2,
        validation_policy: Option<&CurveContext>,
    ) -> Classification<Self> {
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
                    | BezierSplitFragment2::AlgebraicEndpointImages { .. }
                    | BezierSplitFragment2::AnalyticParallel(_)
                    | BezierSplitFragment2::AlgebraicChord(_)
                    | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
                        fragments.push(fragment.fragment().clone());
                    }
                    BezierSplitFragment2::SelectedFiber(_) => {
                        fragments.push(fragment.fragment().clone());
                    }
                    BezierSplitFragment2::Unresolved { .. } => {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
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

    /// Materializes retained region carriers from a resolved linear-overlap traversal.
    ///
    /// The input object already stores both proof stages: exact refinement at
    /// certified linear-overlap endpoints and duplicate-subfragment traversal
    /// over the refined graph.  This constructor keeps that graph/traversal
    /// association intact while accepting both materialized native fragments
    /// and algebraic endpoint-image carriers as retained exact objects.  It
    /// still rejects unresolved carriers, open chains, and invalid refined
    /// indices rather than sampling or repairing them.
    pub fn from_retained_linear_overlap_traversal(
        traversal: &BezierRetainedLinearOverlapTraversal2,
        policy: &CurveContext,
    ) -> CurveOutcome<Classification<Self>> {
        resolve_certified_value(policy, |attempt| {
            Self::from_retained_arrangement_traversal_raw(
                traversal.refinement().graph(),
                traversal.traversal(),
                attempt,
            )
        })
    }

    /// Materializes retained carriers from a represented rational-overlap traversal.
    ///
    /// Native and algebraic endpoint-image fragments remain exact retained
    /// objects; unresolved carriers and open chains remain explicit uncertainty.
    pub fn from_retained_rational_overlap_traversal(
        traversal: &BezierRetainedRationalOverlapTraversal2,
        policy: &CurveContext,
    ) -> CurveOutcome<Classification<Self>> {
        resolve_certified_value(policy, |attempt| {
            Self::from_retained_arrangement_traversal_raw(
                traversal.refinement().graph(),
                traversal.traversal(),
                attempt,
            )
        })
    }

    /// Assigns material/hole roles for retained loops that are exact line images.
    ///
    /// Every retained fragment must either be a materialized polynomial Bezier
    /// that is exactly a degree elevation of its endpoint line segment, or an
    /// algebraic endpoint-image carrier whose contributed endpoints are exact
    /// rational point witnesses. The method lowers those loops to native line
    /// contours and assigns even-odd nesting roles with exact point-in-contour
    /// decisions.  It rejects conics, nonlinear Bezier arcs, algebraic
    /// endpoint-image carriers without exact rational endpoints, unresolved
    /// fragments, boundary-touching loops, and uncertain predicate signs.
    pub fn line_image_role_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<CurveRegionLineRoleEvidence2>>> {
        resolve_certified_operation(policy, |attempt| self.line_image_role_evidence_raw(attempt))
    }

    pub(crate) fn line_image_role_evidence_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveRegionLineRoleEvidence2>> {
        let mut contours = Vec::with_capacity(self.data.boundary_loops.len());
        let mut materialized_fragment_count = 0_usize;
        let mut algebraic_fragment_count = 0_usize;
        for boundary_loop in &self.data.boundary_loops {
            let line_loop = match retained_line_loop_to_contour(boundary_loop, policy)? {
                Classification::Decided(line_loop) => line_loop,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            materialized_fragment_count += line_loop.materialized_fragment_count;
            algebraic_fragment_count += line_loop.algebraic_fragment_count;
            contours.push(line_loop.contour);
        }

        let roles = match retained_line_loop_roles(&contours, policy)? {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let evidence = CurveRegionLineRoleEvidence2::new(
            roles.roles,
            roles.nesting_depths,
            materialized_fragment_count,
            algebraic_fragment_count,
            contours,
        )?
        .with_loop_arrangement_sources(retained_loop_arrangement_sources(
            &self.data.boundary_loops,
        ))?;
        Ok(Classification::Decided(evidence))
    }

    /// Assigns material/hole roles from exact native loop signed-area orientation.
    ///
    /// A negative signed area is treated as a material loop and a positive
    /// signed area as a hole loop, matching the unified region boundary
    /// convention used by [`CurveRegion2::signed_area`].  This method is a
    /// evidence-bearing orientation adapter: it does not infer nesting and it
    /// does not sample nonlinear loops.  Use [`Self::line_image_role_evidence`]
    /// when exact line-image nesting is required.
    pub fn signed_area_role_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<CurveRegionSignedAreaRoleEvidence2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.signed_area_role_evidence_raw(attempt)
        })
    }

    pub(crate) fn signed_area_role_evidence_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveRegionSignedAreaRoleEvidence2>> {
        let mut roles = Vec::with_capacity(self.data.boundary_loops.len());
        let mut signed_areas = Vec::with_capacity(self.data.boundary_loops.len());
        for boundary_loop in &self.data.boundary_loops {
            let area = match boundary_loop.signed_area_raw(policy)? {
                Classification::Decided(Some(area)) => area,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let role = match real_sign(&area, policy) {
                Some(RealSign::Negative) => CurveRegionLoopRole::Material,
                Some(RealSign::Positive) => CurveRegionLoopRole::Hole,
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            roles.push(role);
            signed_areas.push(area);
        }
        let evidence = CurveRegionSignedAreaRoleEvidence2::new(roles, signed_areas, policy)?
            .with_loop_fragment_counts(retained_loop_fragment_counts(&self.data.boundary_loops))?
            .with_loop_arrangement_sources(retained_loop_arrangement_sources(
                &self.data.boundary_loops,
            ))?;
        Ok(Classification::Decided(evidence))
    }

    /// Assigns material/hole roles by exact curved-loop nesting.
    ///
    /// Each retained loop must be fully native and have a nonzero implemented
    /// signed area. The area is used only to reject degenerate/unsupported
    /// loops; role parity comes from exact containment depth. This makes
    /// same-orientation nested nonlinear loops classify as material/hole by
    /// topology instead of by their authored orientation.
    pub fn curved_nesting_role_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<CurveRegionNestingRoleEvidence2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.curved_nesting_role_evidence_raw(attempt)
        })
    }

    pub(crate) fn curved_nesting_role_evidence_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveRegionNestingRoleEvidence2>> {
        let Some(native_loops) = self.native_boundary_loops() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let native_bounds = self.native_boundary_bounds(policy);
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
        let mut nesting_depths = Vec::with_capacity(native_loops.len());
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
            nesting_depths.push(depth);
            roles.push(if depth.is_multiple_of(2) {
                CurveRegionLoopRole::Material
            } else {
                CurveRegionLoopRole::Hole
            });
        }

        let evidence = CurveRegionNestingRoleEvidence2::new(
            roles,
            nesting_depths,
            signed_areas,
            sample_points,
            policy,
        )?
        .with_loop_fragment_counts(retained_loop_fragment_counts(&self.data.boundary_loops))?
        .with_loop_arrangement_sources(retained_loop_arrangement_sources(
            &self.data.boundary_loops,
        ))?;
        Ok(Classification::Decided(evidence))
    }

    /// Returns one exact material/hole role per retained loop.
    ///
    /// The strongest curved nesting classifier is preferred. Exact signed-area
    /// orientation and line-image nesting are retained fallbacks for carrier
    /// subsets that do not support the full curved evidence.
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
        match self.curved_nesting_role_evidence_raw(policy)? {
            Classification::Decided(evidence) => {
                return Ok(Classification::Decided(evidence.roles().to_vec()));
            }
            Classification::Uncertain(_) => {}
        }
        match self.signed_area_role_evidence_raw(policy)? {
            Classification::Decided(evidence) => {
                return Ok(Classification::Decided(evidence.roles().to_vec()));
            }
            Classification::Uncertain(_) => {}
        }
        match self.line_image_role_evidence_raw(policy)? {
            Classification::Decided(evidence) => {
                Ok(Classification::Decided(evidence.roles().to_vec()))
            }
            Classification::Uncertain(UncertaintyReason::Unsupported)
                if self.data.certified_regularized_filled_left_topology =>
            {
                self.regularized_retained_loop_roles_raw(policy)
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Assigns roles to already-regularized retained boundary loops by exact nesting.
    ///
    /// The retained regularization certificate guarantees that distinct
    /// output chains are noncrossing simple boundaries with fill on their
    /// left. A represented interior point of one retained fragment is
    /// therefore inside exactly the loops that contain that complete
    /// boundary. Nesting parity assigns material and hole roles without
    /// requiring a Green integral for procedural analytic parallels.
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
        let evaluators = self.retained_rational_evaluators()?;
        let mut samples = Vec::with_capacity(self.data.boundary_loops.len());
        let mut bounds = Vec::with_capacity(self.data.boundary_loops.len());
        for boundary_loop in &self.data.boundary_loops {
            match retained_loop_sample_point(boundary_loop, policy)? {
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
            for (container_index, (boundary_loop, evaluators)) in self
                .data
                .boundary_loops
                .iter()
                .zip(evaluators.iter())
                .enumerate()
            {
                if candidate_index == container_index {
                    continue;
                }
                if bounds[container_index].as_ref().is_some_and(|bounds| {
                    matches!(
                        bounds.contains_point(sample, policy),
                        Classification::Decided(false)
                    )
                }) {
                    continue;
                }
                match classify_point_against_retained_loop(
                    boundary_loop,
                    evaluators,
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
    /// Roles come from [`Self::loop_roles`]. Each hole contributes a retained
    /// exact endpoint witness which is classified against material carriers;
    /// sampled coordinates and winding are not used. An algebraic endpoint
    /// without a represented point remains explicit uncertainty.
    pub fn boundary_profiles(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Vec<CurveRegionProfile2<'_>>>>> {
        resolve_certified_operation(policy, |attempt| self.boundary_profiles_raw(attempt))
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

        let evaluators = self.retained_rational_evaluators()?;
        let native_loops = self.native_boundary_loops();
        let native_bounds = self.native_boundary_bounds(policy);
        for (hole_index, role) in roles.iter().enumerate() {
            if *role != CurveRegionLoopRole::Hole {
                continue;
            }
            let point =
                match retained_loop_sample_point(&self.data.boundary_loops[hole_index], policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };

            let mut owner: Option<usize> = None;
            for (profile_index, profile) in profiles.iter().enumerate() {
                let material_index = profile.material_loop_index;
                if native_bounds.is_some_and(|bounds| {
                    matches!(
                        bounds[material_index].contains_point(&point, policy),
                        Classification::Decided(false)
                    )
                }) {
                    continue;
                }
                let containment = if let Some(native_loops) = native_loops {
                    classify_point_against_native_loop_after_bounds(
                        &native_loops[material_index],
                        &point,
                        policy,
                    )?
                } else {
                    classify_point_against_retained_loop(
                        profile.material,
                        &evaluators[material_index],
                        &point,
                        policy,
                    )?
                };
                match containment {
                    Classification::Decided(
                        ContourPointLocation::Inside | ContourPointLocation::Boundary,
                    ) => match owner {
                        None => owner = Some(profile_index),
                        Some(owner_index) => {
                            let candidate_point =
                                match retained_loop_sample_point(profile.material, policy)? {
                                    Classification::Decided(point) => point,
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                };
                            let current_owner = &profiles[owner_index];
                            let current_material_index = current_owner.material_loop_index;
                            let candidate_inside_owner = if let Some(native_loops) = native_loops {
                                classify_point_against_native_loop_after_bounds(
                                    &native_loops[current_material_index],
                                    &candidate_point,
                                    policy,
                                )?
                            } else {
                                classify_point_against_retained_loop(
                                    current_owner.material,
                                    &evaluators[current_material_index],
                                    &candidate_point,
                                    policy,
                                )?
                            };
                            match candidate_inside_owner {
                                Classification::Decided(
                                    ContourPointLocation::Inside | ContourPointLocation::Boundary,
                                ) => owner = Some(profile_index),
                                Classification::Decided(ContourPointLocation::Outside) => {
                                    let owner_point = match retained_loop_sample_point(
                                        current_owner.material,
                                        policy,
                                    )? {
                                        Classification::Decided(point) => point,
                                        Classification::Uncertain(reason) => {
                                            return Ok(Classification::Uncertain(reason));
                                        }
                                    };
                                    let owner_inside_candidate =
                                        if let Some(native_loops) = native_loops {
                                            classify_point_against_native_loop_after_bounds(
                                                &native_loops[material_index],
                                                &owner_point,
                                                policy,
                                            )?
                                        } else {
                                            classify_point_against_retained_loop(
                                                profile.material,
                                                &evaluators[material_index],
                                                &owner_point,
                                                policy,
                                            )?
                                        };
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
                match self.materialized_native_line_arc_region(attempt)? {
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
                    match self.line_image_role_evidence_raw(attempt)? {
                        Classification::Decided(evidence) => {
                            let region = self.region_from_line_role_evidence(&evidence)?;
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
    /// second decomposition. Every candidate is rebuilt with this region's
    /// material/hole and fill semantics. Interior algebraic Bezier contacts
    /// retain their selected parameters and exact point images, joined by one
    /// compact algebraic chord instead of falling through to historical
    /// contour machinery.
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
        if mode == CurveCornerMode2::TrimOnly
            && let Some(solutions) = self.retained_chamfer_solutions(
                loop_index,
                vertex_index,
                previous_setback.clone(),
                next_setback.clone(),
                policy,
            )?
        {
            return Ok(solutions);
        }
        let paths =
            match self.materialized_boundary_paths_for_edit(CurveOperation2::Chamfer, policy)? {
                Classification::Decided(paths) => paths,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Chamfer,
                        CurveFamily2::Line,
                        reason,
                    ));
                }
            };
        let path = paths.get(loop_index).ok_or_else(|| {
            curve_region_edit_error(CurveOperation2::Chamfer, CurveError::InvalidCurveRange)
        })?;
        let solutions = path.chamfer_vertex_by_setbacks_raw(
            vertex_index,
            previous_setback,
            next_setback,
            mode,
            policy,
        )?;
        self.rebuild_corner_path_solutions(
            &paths,
            loop_index,
            solutions,
            CurveOperation2::Chamfer,
            policy,
        )
    }

    fn retained_chamfer_solutions(
        &self,
        loop_index: usize,
        vertex_index: usize,
        previous_setback: Real,
        next_setback: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CurveCornerSolutions2<Self>>> {
        let Some(boundary_loop) = self.data.boundary_loops.get(loop_index) else {
            return Err(curve_region_edit_error(
                CurveOperation2::Chamfer,
                CurveError::InvalidCurveRange,
            ));
        };
        let fragment_count = boundary_loop.fragments().len();
        if vertex_index >= fragment_count {
            return Err(curve_region_edit_error(
                CurveOperation2::Chamfer,
                CurveError::InvalidCurveRange,
            ));
        }
        // A one-fragment closed carrier needs two cuts on the same source
        // interval. Keep that distinct case out of this two-incident-fragment
        // fast path until it has a dedicated interval-selection proof.
        if fragment_count < 2 {
            return Ok(None);
        }
        let previous_index = if vertex_index == 0 {
            fragment_count - 1
        } else {
            vertex_index - 1
        };
        let next_index = vertex_index;
        let previous_fragment = &boundary_loop.fragments()[previous_index];
        let next_fragment = &boundary_loop.fragments()[next_index];
        let previous_top_level = match previous_fragment {
            BezierSplitFragment2::Materialized { curve, .. } => Some(Curve2::from(curve.clone())),
            BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
            _ => return Ok(None),
        };
        let next_top_level = match next_fragment {
            BezierSplitFragment2::Materialized { curve, .. } => Some(Curve2::from(curve.clone())),
            BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
            _ => return Ok(None),
        };
        let previous_family = previous_top_level
            .as_ref()
            .map_or(CurveFamily2::RationalBezier, Curve2::family);
        let next_family = next_top_level
            .as_ref()
            .map_or(CurveFamily2::RationalBezier, Curve2::family);
        let previous_sign = validate_corner_design_value(
            &previous_setback,
            CurveOperation2::Chamfer,
            previous_family,
            policy,
        )?;
        let next_sign = validate_corner_design_value(
            &next_setback,
            CurveOperation2::Chamfer,
            next_family,
            policy,
        )?;
        let previous_carrier = match previous_fragment {
            BezierSplitFragment2::Materialized { .. } => exact_corner_carrier(
                previous_top_level
                    .as_ref()
                    .expect("a materialized corner fragment has a top-level carrier"),
                true,
                CurveOperation2::Chamfer,
                policy,
            )?
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Chamfer,
                    previous_family,
                    UncertaintyReason::Unsupported,
                )
            })?,
            BezierSplitFragment2::AlgebraicChord(chord) => {
                crate::curve::ExactCornerCarrier2::AlgebraicChord(chord)
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => {
                crate::curve::ExactCornerCarrier2::AnalyticParallel(fragment)
            }
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                crate::curve::ExactCornerCarrier2::AlgebraicCusp(fragment)
            }
            _ => unreachable!("unsupported retained corner fragments returned above"),
        };
        let next_carrier = match next_fragment {
            BezierSplitFragment2::Materialized { .. } => exact_corner_carrier(
                next_top_level
                    .as_ref()
                    .expect("a materialized corner fragment has a top-level carrier"),
                false,
                CurveOperation2::Chamfer,
                policy,
            )?
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Chamfer,
                    next_family,
                    UncertaintyReason::Unsupported,
                )
            })?,
            BezierSplitFragment2::AlgebraicChord(chord) => {
                crate::curve::ExactCornerCarrier2::AlgebraicChord(chord)
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => {
                crate::curve::ExactCornerCarrier2::AnalyticParallel(fragment)
            }
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                crate::curve::ExactCornerCarrier2::AlgebraicCusp(fragment)
            }
            _ => unreachable!("unsupported retained corner fragments returned above"),
        };
        let solutions = solve_exact_chamfer_corner(
            previous_carrier,
            next_carrier,
            &previous_setback,
            &next_setback,
            previous_sign,
            next_sign,
            CurveCornerMode2::TrimOnly,
            previous_family,
            next_family,
            policy,
        )?;
        try_map_corner_solutions(solutions, |solution| {
            let (previous_cut, next_cut) =
                solution.into_retained_cut_evidence().ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Chamfer,
                        previous_family,
                        UncertaintyReason::Unsupported,
                    )
                })?;
            self.rebuild_retained_chamfer(loop_index, vertex_index, previous_cut, next_cut, policy)
        })
        .map(Some)
    }

    fn rebuild_retained_chamfer(
        &self,
        loop_index: usize,
        vertex_index: usize,
        previous_cut: CornerTrimCut2,
        next_cut: CornerTrimCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let chord = if let (Some(previous_point), Some(next_point)) =
            (previous_cut.point.as_exact(), next_cut.point.as_exact())
        {
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(previous_point.clone(), next_point.clone()).map_err(
                        |cause| curve_region_edit_error(CurveOperation2::Chamfer, cause),
                    )?,
                )),
            }
        } else {
            match crate::BezierAlgebraicChord2::try_new(
                previous_cut.point.clone(),
                next_cut.point.clone(),
                policy,
            )
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Chamfer, cause))?
            {
                Classification::Decided(chord) => BezierSplitFragment2::AlgebraicChord(chord),
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Chamfer,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            }
        };
        self.rebuild_retained_corner(
            loop_index,
            vertex_index,
            previous_cut,
            next_cut,
            vec![chord],
            CurveOperation2::Chamfer,
            policy,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn rebuild_retained_corner(
        &self,
        loop_index: usize,
        vertex_index: usize,
        previous_cut: CornerTrimCut2,
        next_cut: CornerTrimCut2,
        inserted: Vec<BezierSplitFragment2>,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let boundary_loop = &self.data.boundary_loops[loop_index];
        let fragment_count = boundary_loop.fragments().len();
        let previous_index = if vertex_index == 0 {
            fragment_count - 1
        } else {
            vertex_index - 1
        };
        let next_index = vertex_index;
        let previous_trim = if previous_cut.trim {
            retained_corner_fragment_trim(
                &boundary_loop.fragments()[previous_index],
                previous_cut.parameter,
                &previous_cut.point,
                previous_cut.replacement_rational_curve.as_ref(),
                true,
                operation,
                policy,
            )?
        } else {
            boundary_loop.fragments()[previous_index].clone()
        };
        let next_trim = if next_cut.trim {
            retained_corner_fragment_trim(
                &boundary_loop.fragments()[next_index],
                next_cut.parameter,
                &next_cut.point,
                next_cut.replacement_rational_curve.as_ref(),
                false,
                operation,
                policy,
            )?
        } else {
            boundary_loop.fragments()[next_index].clone()
        };

        let mut fragments = Vec::with_capacity(fragment_count + inserted.len());
        if vertex_index == 0 {
            fragments.extend(inserted);
            fragments.push(next_trim);
            fragments.extend(
                boundary_loop.fragments()[next_index + 1..previous_index]
                    .iter()
                    .cloned(),
            );
            fragments.push(previous_trim);
        } else {
            fragments.extend(boundary_loop.fragments()[..previous_index].iter().cloned());
            fragments.push(previous_trim);
            fragments.extend(inserted);
            fragments.push(next_trim);
            fragments.extend(boundary_loop.fragments()[next_index + 1..].iter().cloned());
        }
        let edited_loop = CurveRegionBoundaryLoop2::new(fragments, policy)
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
        Self::try_new_with_loop_topology(loops, roles, fill_rules, interior_sides)
            .map_err(|cause| curve_region_edit_error(operation, cause))
    }

    /// Solves a boundary-loop circular fillet from an exact radius.
    ///
    /// Exact candidates come from the same carrier-interaction authority used
    /// by open [`CurvePath2`] editing and are rebuilt without changing loop role
    /// or fill rule. Represented direct and canonical spline/NURBS Bezier trims
    /// use the retained path authority.
    pub fn fillet_loop_vertex_by_radius(
        &self,
        loop_index: usize,
        vertex_index: usize,
        radius: Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveCornerSolutions2<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            self.fillet_loop_vertex_by_radius_raw(loop_index, vertex_index, radius, mode, attempt)
        })
    }

    fn fillet_loop_vertex_by_radius_raw(
        &self,
        loop_index: usize,
        vertex_index: usize,
        radius: Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveCornerSolutions2<Self>> {
        if mode == CurveCornerMode2::TrimOnly
            && let Some(solutions) =
                self.retained_fillet_solutions(loop_index, vertex_index, radius.clone(), policy)?
        {
            return Ok(solutions);
        }
        let paths =
            match self.materialized_boundary_paths_for_edit(CurveOperation2::Fillet, policy)? {
                Classification::Decided(paths) => paths,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::Line,
                        reason,
                    ));
                }
            };
        let path = paths.get(loop_index).ok_or_else(|| {
            curve_region_edit_error(CurveOperation2::Fillet, CurveError::InvalidCurveRange)
        })?;
        let solutions = path.fillet_vertex_by_radius_raw(vertex_index, radius, mode, policy)?;
        self.rebuild_corner_path_solutions(
            &paths,
            loop_index,
            solutions,
            CurveOperation2::Fillet,
            policy,
        )
    }

    fn retained_fillet_solutions(
        &self,
        loop_index: usize,
        vertex_index: usize,
        radius: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CurveCornerSolutions2<Self>>> {
        let Some(boundary_loop) = self.data.boundary_loops.get(loop_index) else {
            return Err(curve_region_edit_error(
                CurveOperation2::Fillet,
                CurveError::InvalidCurveRange,
            ));
        };
        let fragment_count = boundary_loop.fragments().len();
        if vertex_index >= fragment_count {
            return Err(curve_region_edit_error(
                CurveOperation2::Fillet,
                CurveError::InvalidCurveRange,
            ));
        }
        if fragment_count < 2 {
            return Ok(None);
        }
        let previous_index = if vertex_index == 0 {
            fragment_count - 1
        } else {
            vertex_index - 1
        };
        let next_index = vertex_index;
        let operation_curve = |fragment: &BezierSplitFragment2| match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => Some(Curve2::from(curve.clone())),
            BezierSplitFragment2::AlgebraicChord(chord) => chord.exact_line().map(Curve2::from),
            _ => None,
        };
        let previous_fragment = &boundary_loop.fragments()[previous_index];
        let next_fragment = &boundary_loop.fragments()[next_index];
        let previous_curve = operation_curve(previous_fragment);
        let next_curve = operation_curve(next_fragment);
        let previous_is_cusp = matches!(
            previous_fragment,
            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
        );
        let next_is_cusp = matches!(
            next_fragment,
            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
        );
        let previous_is_chord =
            matches!(previous_fragment, BezierSplitFragment2::AlgebraicChord(_));
        let next_is_chord = matches!(next_fragment, BezierSplitFragment2::AlgebraicChord(_));
        let previous_is_parallel =
            matches!(previous_fragment, BezierSplitFragment2::AnalyticParallel(_));
        let next_is_parallel = matches!(next_fragment, BezierSplitFragment2::AnalyticParallel(_));
        if previous_curve.is_none()
            && !previous_is_cusp
            && !previous_is_chord
            && !previous_is_parallel
        {
            return Ok(None);
        }
        if next_curve.is_none() && !next_is_cusp && !next_is_chord && !next_is_parallel {
            return Ok(None);
        }
        let previous_family = previous_curve
            .as_ref()
            .map_or(CurveFamily2::RationalBezier, Curve2::family);
        let next_family = next_curve
            .as_ref()
            .map_or(CurveFamily2::RationalBezier, Curve2::family);
        let radius_sign = validate_corner_design_value(
            &radius,
            CurveOperation2::Fillet,
            previous_family,
            policy,
        )?;
        if radius_sign == RealSign::Zero {
            return Ok(Some(CurveCornerSolutions2::NoSolution(
                crate::CurveCornerNoSolution2::ZeroDesignValue,
            )));
        }
        let previous_carrier = if let Some(previous_curve) = &previous_curve {
            exact_corner_carrier(previous_curve, true, CurveOperation2::Fillet, policy)?
                .ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        UncertaintyReason::Unsupported,
                    )
                })?
        } else {
            {
                match previous_fragment {
                    BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                        crate::curve::ExactCornerCarrier2::AlgebraicCusp(fragment)
                    }
                    BezierSplitFragment2::AlgebraicChord(chord) => {
                        crate::curve::ExactCornerCarrier2::AlgebraicChord(chord)
                    }
                    BezierSplitFragment2::AnalyticParallel(fragment) => {
                        crate::curve::ExactCornerCarrier2::AnalyticParallel(fragment)
                    }
                    _ => unreachable!("the supported retained fillet carrier is closed"),
                }
            }
        };
        let next_carrier = if let Some(next_curve) = &next_curve {
            exact_corner_carrier(next_curve, false, CurveOperation2::Fillet, policy)?.ok_or_else(
                || {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        next_family,
                        UncertaintyReason::Unsupported,
                    )
                },
            )?
        } else {
            {
                match next_fragment {
                    BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                        crate::curve::ExactCornerCarrier2::AlgebraicCusp(fragment)
                    }
                    BezierSplitFragment2::AlgebraicChord(chord) => {
                        crate::curve::ExactCornerCarrier2::AlgebraicChord(chord)
                    }
                    BezierSplitFragment2::AnalyticParallel(fragment) => {
                        crate::curve::ExactCornerCarrier2::AnalyticParallel(fragment)
                    }
                    _ => unreachable!("the supported retained fillet carrier is closed"),
                }
            }
        };
        let solutions = solve_exact_fillet_corner(
            previous_carrier,
            next_carrier,
            &radius,
            radius_sign,
            CurveCornerMode2::TrimOnly,
            previous_family,
            next_family,
            policy,
        )?;
        try_map_corner_solutions(solutions, |solution| {
            let (mut previous_cut, mut next_cut, center, clockwise, retained_frame) =
                solution.into_retained_cut_evidence().ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        UncertaintyReason::Unsupported,
                    )
                })?;
            self.canonicalize_retained_fillet_cut(
                &boundary_loop.fragments()[previous_index],
                &mut previous_cut,
                policy,
            )?;
            self.canonicalize_retained_fillet_cut(
                &boundary_loop.fragments()[next_index],
                &mut next_cut,
                policy,
            )?;
            let inserted = self.retained_fillet_fragments(
                previous_fragment,
                next_fragment,
                &mut previous_cut,
                &mut next_cut,
                center,
                clockwise,
                retained_frame,
                &radius,
                policy,
            )?;
            self.rebuild_retained_corner(
                loop_index,
                vertex_index,
                previous_cut,
                next_cut,
                inserted,
                CurveOperation2::Fillet,
                policy,
            )
        })
        .map(Some)
    }

    fn retained_fillet_sweep(
        frame: &RetainedFilletFrame2,
        other_parallel: &BezierParallel2,
        other_parameter: &BezierParameter2,
        other_reversed: bool,
        fillet_clockwise: bool,
        policy: &CurveContext,
    ) -> ExactCurveResult<(u8, RealSign, RealSign)> {
        let (tangent_cross, tangent_dot) = if let Some(relation) = frame.anchor_evidence.as_ref() {
            match (relation.cross, relation.dot) {
                (Some(cross), dot) => (cross, dot.unwrap_or(RealSign::Zero)),
                (None, Some(dot)) => (RealSign::Zero, dot),
                (None, None) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        UncertaintyReason::Predicate,
                    ));
                }
            }
        } else {
            let RetainedFilletRadialFrame2::RepresentedUnitNormal(unit_normal) =
                &frame.radial_frame
            else {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                ));
            };
            let line_tangent = (unit_normal.1.clone(), -unit_normal.0.clone());
            let (mut tangent_cross, mut tangent_dot) = match other_parallel
                .vector_tangent_cross_and_dot_signs(
                    other_parameter,
                    &line_tangent.0,
                    &line_tangent.1,
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(signs) => signs,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            };
            if other_reversed {
                tangent_cross = exact_sign_reverse(tangent_cross);
                tangent_dot = exact_sign_reverse(tangent_dot);
            }
            (tangent_cross, tangent_dot)
        };
        let sweep_halves = match (fillet_clockwise, tangent_cross) {
            (false, RealSign::Positive) | (true, RealSign::Negative) => 1_u8,
            (false, RealSign::Negative) | (true, RealSign::Positive) => 2_u8,
            (_, RealSign::Zero) if tangent_dot == RealSign::Negative => 1_u8,
            (_, RealSign::Zero) if tangent_dot == RealSign::Positive => {
                return Err(curve_region_edit_error(
                    CurveOperation2::Fillet,
                    CurveError::Topology(
                        "distinct fillet contacts retained the same oriented tangent".into(),
                    ),
                ));
            }
            (_, RealSign::Zero) => {
                return Err(curve_region_edit_error(
                    CurveOperation2::Fillet,
                    CurveError::Topology(
                        "regular fillet tangents had zero cross and dot products".into(),
                    ),
                ));
            }
        };
        Ok((sweep_halves, tangent_cross, tangent_dot))
    }

    #[allow(clippy::too_many_arguments)]
    fn retained_parallel_fillet_fragments(
        frame: &RetainedFilletFrame2,
        fillet: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        other_parallel: &BezierParallel2,
        other_parameter: BezierParameter2,
        other_reversed: bool,
        fillet_clockwise: bool,
        anchor_cut: &mut CornerTrimCut2,
        other_cut: &mut CornerTrimCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        other_cut.parameter = CurveRegionParameter2::from_bezier(other_parameter.clone());
        let (sweep_halves, tangent_cross, tangent_dot) = Self::retained_fillet_sweep(
            frame,
            other_parallel,
            &other_parameter,
            other_reversed,
            fillet_clockwise,
            policy,
        )?;
        let terminal_circle = if sweep_halves == 2 {
            fillet.complementary_half()
        } else {
            fillet.clone()
        };
        let fillet_parameter = if tangent_cross == RealSign::Zero {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        } else if terminal_circle.uses_selected_parallel_normal_frame() {
            let derivative_scale = match other_parallel
                .parallel_derivative_scale_sign(&other_parameter, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        UncertaintyReason::Boundary,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            };
            let traversal_agrees_with_source =
                (derivative_scale == RealSign::Positive) != other_reversed;
            let signed_offset_sign = if fillet_clockwise {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            let other_radial_sign = if traversal_agrees_with_source {
                exact_sign_reverse(signed_offset_sign)
            } else {
                signed_offset_sign
            };
            let frame_cross = if frame.anchor_is_previous {
                tangent_cross
            } else {
                exact_sign_reverse(tangent_cross)
            };
            match terminal_circle
                .certified_selected_parallel_contact_parameter(
                    other_parallel.clone(),
                    other_parameter.clone(),
                    other_radial_sign,
                    frame_cross,
                    tangent_dot,
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            }
        } else {
            let parameter_map = match terminal_circle
                .parallel_parameter_map(other_parallel, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(parameter_map) => parameter_map,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            };
            parameter_map.certified_interior_tangent_parameter(other_parameter)
        };
        other_cut.point = match fillet_parameter
            .coincident_point_evidence(&terminal_circle, policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
        {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::CircularArc,
                    UncertaintyReason::Unsupported,
                ));
            }
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::CircularArc,
                    reason,
                ));
            }
        };
        anchor_cut.point = match fillet
            .start_point_evidence(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::CircularArc,
                    reason,
                ));
            }
        };

        let exact_zero =
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero());
        let exact_one =
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one());
        let mut fragments = if sweep_halves == 1 {
            vec![
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    fillet,
                    exact_zero,
                    fillet_parameter,
                    false,
                    policy,
                ),
            ]
        } else {
            vec![
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    fillet,
                    exact_zero.clone(),
                    exact_one,
                    false,
                    policy,
                ),
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    terminal_circle,
                    exact_zero,
                    fillet_parameter,
                    false,
                    policy,
                ),
            ]
        };
        if !frame.anchor_is_previous {
            fragments = fragments
                .into_iter()
                .rev()
                .map(|fragment| fragment.reversed())
                .collect();
        }
        Ok(fragments
            .into_iter()
            .map(BezierSplitFragment2::AlgebraicCuspSemicircle)
            .collect())
    }

    #[allow(clippy::too_many_arguments)]
    fn retained_fillet_fragments(
        &self,
        previous_fragment: &BezierSplitFragment2,
        next_fragment: &BezierSplitFragment2,
        previous_cut: &mut CornerTrimCut2,
        next_cut: &mut CornerTrimCut2,
        center: RationalBezierIntersectionPointEvidence2,
        clockwise: bool,
        retained_frame: Option<RetainedFilletFrame2>,
        radius: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        if let (Some(previous_point), Some(next_point), Some(center)) = (
            previous_cut.point.as_exact(),
            next_cut.point.as_exact(),
            center.as_exact(),
        ) {
            let arc = CircularArc2::new_with_certified_radius(
                previous_point.clone(),
                next_point.clone(),
                center.clone(),
                radius * radius,
                clockwise,
                None,
            );
            let decomposition = match crate::arc_bezier::decompose_circular_arc(&arc, policy)? {
                Classification::Decided(decomposition) => decomposition,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            };
            return Ok(decomposition
                .spans()
                .iter()
                .map(|span| BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::RationalQuadratic(span.curve().clone()),
                })
                .collect());
        }

        {
            let frame = retained_frame.ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                )
            })?;
            let (anchor_cut, other_cut, other_fragment) = if frame.anchor_is_previous {
                (&mut *previous_cut, &mut *next_cut, next_fragment)
            } else {
                (&mut *next_cut, &mut *previous_cut, previous_fragment)
            };
            if let Some(relation) = frame.anchor_evidence.as_ref() {
                anchor_cut.replacement_rational_curve = relation.canonical_anchor_curve.clone();
            }
            if !matches!(
                other_fragment,
                BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                    | BezierSplitFragment2::AnalyticParallel(_)
                    | BezierSplitFragment2::Materialized { .. }
            ) {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                ));
            }
            let fillet_clockwise = if frame.anchor_is_previous {
                clockwise
            } else {
                !clockwise
            };
            let fillet = match match &frame.radial_frame {
                RetainedFilletRadialFrame2::RepresentedUnitNormal(unit_normal) => {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_certified_unit_normal(
                        &center,
                        unit_normal.clone(),
                        frame.radial_distance.clone(),
                        fillet_clockwise,
                        policy,
                    )
                }
                RetainedFilletRadialFrame2::ConcentricArc {
                    support_center,
                    normal_denominator,
                } => crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_certified_concentric_normal(
                    &center,
                    support_center,
                    normal_denominator.clone(),
                    frame.radial_distance.clone(),
                    fillet_clockwise,
                    policy,
                ),
                RetainedFilletRadialFrame2::ParallelNormal {
                    center_support,
                    center_parameter,
                    policy: frame_policy,
                } => {
                    if frame_policy != policy {
                        return Err(curve_region_edit_error(
                            CurveOperation2::Fillet,
                            CurveError::Topology(
                                "a parallel-normal fillet frame crossed predicate policies".into(),
                            ),
                        ));
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                        center_support.clone(),
                        center_parameter.clone(),
                        frame.radial_distance.clone(),
                        fillet_clockwise,
                        policy,
                    )
                }
            }
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))? {
                Classification::Decided(Some(fillet)) => fillet,
                Classification::Decided(None) => {
                    return Err(curve_region_edit_error(
                        CurveOperation2::Fillet,
                        CurveError::Topology("a positive-radius retained fillet collapsed".into()),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            };
            let fillet_parameter_is_open = |fillet_parameter: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2| {
                let after_start = match fillet_parameter.order_to_real(&Real::zero(), policy) {
                    Ok(Classification::Decided(order)) => order == std::cmp::Ordering::Greater,
                    Ok(Classification::Uncertain(reason)) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            CurveFamily2::CircularArc,
                            reason,
                        ));
                    }
                    Err(cause) => {
                        return Err(curve_region_edit_error(CurveOperation2::Fillet, cause));
                    }
                };
                let before_end = match fillet_parameter.order_to_real(&Real::one(), policy) {
                    Ok(Classification::Decided(order)) => order == std::cmp::Ordering::Less,
                    Ok(Classification::Uncertain(reason)) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            CurveFamily2::CircularArc,
                            reason,
                        ));
                    }
                    Err(cause) => {
                        return Err(curve_region_edit_error(CurveOperation2::Fillet, cause));
                    }
                };
                Ok(after_start && before_end)
            };
            let fillet_parameter = match other_fragment {
                BezierSplitFragment2::AlgebraicCuspSemicircle(other_fragment) => {
                    let intersections = match fillet
                        .pair_intersections(other_fragment.semicircle(), policy)
                        .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
                    {
                        Classification::Decided(intersections) => intersections,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                CurveFamily2::CircularArc,
                                reason,
                            ));
                        }
                    };
                    let mut contacts = Vec::new();
                    match intersections {
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                            contacts: pair_contacts,
                            parameter_map,
                        } => {
                            contacts.reserve(pair_contacts.len());
                            for contact in pair_contacts {
                                contacts.push((
                                    parameter_map.first_contact_parameter(&contact),
                                    parameter_map.second_contact_parameter(&contact),
                                ));
                            }
                        }
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(
                            pair_contacts,
                        ) => {
                            let endpoint = |location| match location {
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Start => crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End => crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one()),
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior => unreachable!("endpoint-only pair contacts are not interior"),
                            };
                            contacts.reserve(pair_contacts.len());
                            for contact in pair_contacts {
                                contacts.push((
                                    endpoint(contact.first_location),
                                    endpoint(contact.second_location),
                                ));
                            }
                        }
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts => {}
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(_) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                CurveFamily2::CircularArc,
                                UncertaintyReason::Boundary,
                            ));
                        }
                    }
                    if contacts.len() != 1 {
                        return Err(curve_region_edit_error(
                            CurveOperation2::Fillet,
                            CurveError::Topology(
                                "a retained fillet radius-offset replay was not a unique circle tangency"
                                    .into(),
                            ),
                        ));
                    }
                    let (fillet_parameter, other_parameter) = contacts
                        .pop()
                        .expect("one retained circle tangency was certified");
                    if !fillet_parameter_is_open(&fillet_parameter)? {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            CurveFamily2::CircularArc,
                            UncertaintyReason::Boundary,
                        ));
                    }
                    match other_fragment
                        .contains_parameter(&other_parameter, false, false, policy)
                        .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
                    {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                CurveFamily2::CircularArc,
                                UncertaintyReason::Boundary,
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                CurveFamily2::CircularArc,
                                reason,
                            ));
                        }
                    }
                    other_cut.parameter =
                        CurveRegionParameter2::from_algebraic_cusp(other_parameter);
                    fillet_parameter
                }
                BezierSplitFragment2::AnalyticParallel(other_fragment) => {
                    let expected_parameter = other_cut
                        .parameter
                        .as_bezier_parameter()
                        .cloned()
                        .ok_or_else(|| {
                            ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                CurveFamily2::RationalBezier,
                                UncertaintyReason::Unsupported,
                            )
                        })?;
                    match crate::bezier_offset::overlap_parameter_is_in_range(
                        &expected_parameter,
                        other_fragment.range(),
                        false,
                        policy,
                    )
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
                    {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => {
                            return Err(curve_region_edit_error(
                                CurveOperation2::Fillet,
                                CurveError::Topology(
                                    "a retained analytic fillet lost its interior trim parameter"
                                        .into(),
                                ),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                CurveFamily2::RationalBezier,
                                reason,
                            ));
                        }
                    }
                    return Self::retained_parallel_fillet_fragments(
                        &frame,
                        fillet,
                        other_fragment.parallel(),
                        expected_parameter,
                        other_fragment.is_reversed(),
                        fillet_clockwise,
                        anchor_cut,
                        other_cut,
                        policy,
                    );
                }
                BezierSplitFragment2::Materialized { curve, .. } => {
                    let expected_parameter = other_cut
                        .parameter
                        .as_bezier_parameter()
                        .cloned()
                        .ok_or_else(|| {
                            ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                CurveFamily2::RationalBezier,
                                UncertaintyReason::Unsupported,
                            )
                        })?;
                    let rational = RationalBezier2::try_from_subcurve(curve)
                        .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?;
                    let source_parallel = rational
                        .parallel_left(Real::zero())
                        .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?;
                    return Self::retained_parallel_fillet_fragments(
                        &frame,
                        fillet,
                        &source_parallel,
                        expected_parameter,
                        false,
                        fillet_clockwise,
                        anchor_cut,
                        other_cut,
                        policy,
                    );
                }
                _ => unreachable!("the retained fillet companion family was checked"),
            };
            let shared_point = match fillet_parameter
                .coincident_point_evidence(&fillet, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        UncertaintyReason::Unsupported,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            };
            other_cut.point = shared_point;
            anchor_cut.point = match fillet
                .start_point_evidence(policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            };

            let fragment = match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                fillet,
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                fillet_parameter,
                !frame.anchor_is_previous,
                policy,
            )
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(fragment) => fragment,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::CircularArc,
                        reason,
                    ));
                }
            };
            Ok(vec![BezierSplitFragment2::AlgebraicCuspSemicircle(
                fragment,
            )])
        }
    }

    fn canonicalize_retained_fillet_cut(
        &self,
        fragment: &BezierSplitFragment2,
        cut: &mut CornerTrimCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<()> {
        if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
            cut.parameter = match chord
                .parameter_at_certified_point(cut.point.clone(), policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?
            {
                Classification::Decided(Some(parameter)) => {
                    CurveRegionParameter2::from_algebraic_chord(parameter)
                }
                Classification::Decided(None) => {
                    return Err(curve_region_edit_error(
                        CurveOperation2::Fillet,
                        CurveError::Topology(
                            "an exact retained fillet cut lay outside its chord".into(),
                        ),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            };
        }
        Ok(())
    }

    /// Chamfers one boundary-loop vertex without leaving the unified carrier.
    ///
    /// `loop_index` addresses this region's retained boundary order. The edited
    /// Fully materialized loops use exact `CurvePath2` subdivision and are
    /// rebuilt with their material/hole and fill semantics intact.
    /// Algebraic-endpoint fragments that cannot be materialized remain explicit
    /// `Unsupported` uncertainty.
    pub fn chamfer_loop_vertex_by_parameters(
        &self,
        loop_index: usize,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            self.chamfer_loop_vertex_by_parameters_raw(
                loop_index,
                vertex_index,
                previous_param,
                next_param,
                attempt,
            )
        })
    }

    fn chamfer_loop_vertex_by_parameters_raw(
        &self,
        loop_index: usize,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Self>> {
        let mut paths =
            match self.materialized_boundary_paths_for_edit(CurveOperation2::Chamfer, policy)? {
                Classification::Decided(paths) => paths,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let path = paths.get(loop_index).ok_or_else(|| {
            curve_region_edit_error(CurveOperation2::Chamfer, CurveError::InvalidCurveRange)
        })?;
        paths[loop_index] = match path.chamfer_vertex_by_parameters_raw(
            vertex_index,
            previous_param,
            next_param,
            policy,
        ) {
            Ok(path) => path,
            Err(ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
            Err(error) => return Err(error),
        };
        self.rebuild_after_materialized_path_edit(paths, CurveOperation2::Chamfer, policy)
    }

    /// Fillets one boundary-loop vertex without leaving the unified carrier.
    ///
    /// The exact trim parameters, center, and sweep direction are validated by
    /// exact `CurvePath2` editing.
    /// Successful output preserves material/hole and fill semantics.
    #[allow(clippy::too_many_arguments)]
    pub fn fillet_loop_vertex_by_parameters(
        &self,
        loop_index: usize,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        center: &Point2,
        clockwise: bool,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            self.fillet_loop_vertex_by_parameters_raw(
                loop_index,
                vertex_index,
                previous_param,
                next_param,
                center,
                clockwise,
                attempt,
            )
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn fillet_loop_vertex_by_parameters_raw(
        &self,
        loop_index: usize,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        center: &Point2,
        clockwise: bool,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Self>> {
        let mut paths =
            match self.materialized_boundary_paths_for_edit(CurveOperation2::Fillet, policy)? {
                Classification::Decided(paths) => paths,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let path = paths.get(loop_index).ok_or_else(|| {
            curve_region_edit_error(CurveOperation2::Fillet, CurveError::InvalidCurveRange)
        })?;
        paths[loop_index] = match path.fillet_vertex_by_parameters_raw(
            vertex_index,
            previous_param,
            next_param,
            center,
            clockwise,
            policy,
        ) {
            Ok(path) => path,
            Err(ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
            Err(error) => return Err(error),
        };
        self.rebuild_after_materialized_path_edit(paths, CurveOperation2::Fillet, policy)
    }

    fn materialized_boundary_paths_for_edit(
        &self,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Vec<CurvePath2>>> {
        let mut paths = Vec::with_capacity(self.data.boundary_loops.len());
        for boundary_loop in &self.data.boundary_loops {
            let mut curves = Vec::with_capacity(boundary_loop.fragments().len());
            for fragment in boundary_loop.fragments() {
                let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                curves.push(Curve2::from(curve.clone()));
            }
            let path = if self.data.strict_materialized_connectivity_certified {
                CurvePath2::from_structurally_closed_curves(curves)
            } else {
                let path = CurvePath2::try_new_raw(curves, policy)
                    .map_err(|error| error.with_operation(operation))?;
                match crate::curve::validate_closed_curve_path_connectivity(&path, policy)
                    .map_err(|error| error.with_operation(operation))?
                {
                    Classification::Decided(()) => path,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            paths.push(path);
        }
        Ok(Classification::Decided(paths))
    }

    fn rebuild_corner_path_solutions(
        &self,
        paths: &[CurvePath2],
        loop_index: usize,
        solutions: CurveCornerSolutions2<CurvePath2>,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveCornerSolutions2<Self>> {
        let rebuild = |path: CurvePath2| -> ExactCurveResult<Self> {
            let family = path.curves()[0].family();
            let mut edited_paths = paths.to_vec();
            edited_paths[loop_index] = path;
            match self.rebuild_after_materialized_path_edit(edited_paths, operation, policy)? {
                Classification::Decided(region) => Ok(region),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            }
        };
        try_map_corner_solutions(solutions, rebuild)
    }

    /// Materializes every retained boundary loop as an exact top-level path.
    ///
    /// Native and represented polynomial/rational fragments preserve their
    /// exact canonical curve carriers and local parameters. Authored spline and
    /// NURBS curves were intentionally collapsed to their exact native Bezier
    /// spans during region construction; this method does not reconstruct the
    /// larger authored carrier. Regions whose traversal still contains an
    /// algebraic endpoint that cannot be represented by a public [`Curve2`]
    /// return explicit `Unsupported` uncertainty rather than segmenting the
    /// boundary. This is the lossless interchange counterpart to
    /// [`CurveRegion2::project_to_finite_profiles`]. The returned
    /// [`CurveOutcome`] records whether validating exact joins consumed the
    /// `APPROXIMATE_512` terminal.
    pub fn materialized_boundary_paths(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<Vec<CurvePath2>>>> {
        resolve_certified_operation(policy, |attempt| {
            self.materialized_boundary_paths_for_edit(CurveOperation2::NativeTopology, attempt)
        })
    }

    fn rebuild_after_materialized_path_edit(
        &self,
        paths: Vec<CurvePath2>,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Self>> {
        let paths = paths
            .into_iter()
            .map(canonicalize_retained_quadratics_in_corner_path)
            .collect::<Vec<_>>();
        let roles = match self
            .loop_roles_raw(policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let fill_rules = self
            .data
            .certified_loop_fill_rules
            .as_deref()
            .map_or_else(|| vec![FillRule::EvenOdd; paths.len()], <[_]>::to_vec);
        Self::try_from_boundary_paths_with_loop_semantics_raw(
            &paths,
            &roles,
            &fill_rules,
            policy,
            None,
        )
        .map_err(|error| error.with_operation(operation))
        .map(Classification::Decided)
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
        let paths = match self
            .materialized_boundary_paths_for_edit(CurveOperation2::Subdivision, policy)?
        {
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

        let region = Self::try_from_native_contours_raw(material, holes, policy)?;
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

    /// Offsets every boundary so positive distance expands the filled region.
    ///
    /// The exact filled side of each loop selects the required signed left
    /// offset: material exteriors move away from fill while hole boundaries
    /// move into their voids. Independently offset material components and
    /// voids are unioned, then the unified void set is subtracted, so overlaps
    /// created by expansion are returned as regularized boundary topology.
    /// Native line/arc contours retain their specialized wavefront path.
    /// Materialized polynomial and rational spans lower to exact analytic
    /// parallels, split at every certified offset cusp, and retain exact PH
    /// materializations where available. Round joins are exact circular conics;
    /// bevel and bounded-miter joins are exact lines. Every general result then
    /// passes through the same authoritative exact arrangement that removes
    /// self-walk branches and composes material and hole loops. Certified convex
    /// all-line contractions and orthogonal non-convex collapses retain their
    /// faster exact native paths. Unsupported retained source fragments and the
    /// remaining non-orthogonal post-collapse wavefront cases return explicit
    /// uncertainty rather than sampled geometry.
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
        self.offset_exact_general_raw(distance, corner_style, policy)
    }

    fn try_offset_native_topology_fast_path(
        &self,
        distance: Real,
        corner_style: &OffsetCornerStyle2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Self>> {
        if is_zero(&distance, policy) == Some(true) {
            return Ok(Classification::Decided(self.clone()));
        }
        let distance_positive = match real_sign(&distance, policy) {
            Some(RealSign::Positive) => true,
            Some(RealSign::Negative) => false,
            Some(RealSign::Zero) => return Ok(Classification::Decided(self.clone())),
            None => {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            }
        };
        let region = match self
            .native_line_arc_region(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let roles = match self
            .loop_roles_raw(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
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
        if roles.len() != self.data.boundary_loops.len()
            || filled_sides.len() != self.data.boundary_loops.len()
        {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology(
                    "curve-region offset semantics are inconsistent with boundary loops".into(),
                ),
            ));
        }

        let mut material_components = Vec::new();
        let mut void_components = Vec::new();
        for (loop_index, (role, filled_side_is_left)) in
            roles.iter().zip(filled_sides.iter()).enumerate()
        {
            let ordinal = roles[..loop_index]
                .iter()
                .filter(|candidate| *candidate == role)
                .count();
            let contour = native_region_role_contour(region, *role, ordinal)
                .expect("validated native region role inventory")
                .clone();
            let signed_left_distance = if *filled_side_is_left {
                Real::zero() - &distance
            } else {
                distance.clone()
            };
            let component_expands = (*role == CurveRegionLoopRole::Material) == distance_positive;
            let all_line_source = contour
                .segments()
                .iter()
                .all(|segment| matches!(segment, Segment2::Line(_)));
            if !component_expands && all_line_source {
                match contour
                    .offset_left_orthogonal_line_erosion(signed_left_distance.clone(), policy)
                    .map_err(|error| error.with_operation(CurveOperation2::Offset))?
                {
                    Classification::Decided(region) => {
                        push_native_offset_component(
                            *role,
                            region,
                            &mut material_components,
                            &mut void_components,
                        );
                        continue;
                    }
                    Classification::Uncertain(
                        UncertaintyReason::Unsupported
                        | UncertaintyReason::RealSign
                        | UncertaintyReason::Ordering,
                    ) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let raw_offset = match contour
                .offset_left_with_corner_style(signed_left_distance.clone(), corner_style, policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(offset) => offset,
                Classification::Uncertain(reason) => {
                    if !component_expands && all_line_source {
                        match contour
                            .offset_left_convex_line_erosion(signed_left_distance.clone(), policy)
                            .map_err(|cause| {
                                curve_region_edit_error(CurveOperation2::Offset, cause)
                            })? {
                            Classification::Decided(contour) => {
                                push_native_offset_component(
                                    *role,
                                    curve_region_from_optional_native_material_contour(
                                        contour, policy,
                                    )?,
                                    &mut material_components,
                                    &mut void_components,
                                );
                                continue;
                            }
                            Classification::Uncertain(UncertaintyReason::Unsupported) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let self_contacts = raw_offset
                .has_self_contacts(policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
            if !component_expands && all_line_source {
                match raw_offset
                    .retained_contracting_line_offset_cycles(policy)
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                {
                    Classification::Decided(cycles) => {
                        push_native_offset_component(
                            *role,
                            regularize_native_cycles_with_curve_region(
                                cycles,
                                raw_offset.fill_rule(),
                                policy,
                            )?,
                            &mut material_components,
                            &mut void_components,
                        );
                        continue;
                    }
                    Classification::Uncertain(
                        UncertaintyReason::Unsupported
                        | UncertaintyReason::RealSign
                        | UncertaintyReason::Ordering,
                    ) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            if !component_expands
                && all_line_source
                && (!raw_offset.has_retained_regular_offset_branch()
                    || !matches!(self_contacts, Classification::Decided(false)))
            {
                match contour
                    .offset_left_convex_line_erosion(signed_left_distance, policy)
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                {
                    Classification::Decided(contour) => {
                        push_native_offset_component(
                            *role,
                            curve_region_from_optional_native_material_contour(contour, policy)?,
                            &mut material_components,
                            &mut void_components,
                        );
                        continue;
                    }
                    Classification::Uncertain(UncertaintyReason::Unsupported) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            if !component_expands
                && all_line_source
                && !raw_offset.has_retained_regular_offset_branch()
            {
                // A self-contact-free non-orthogonal polygonal parallel can
                // reappear after the wavefront has collapsed. Joined offset
                // construction retains provenance only while every output edge
                // still follows its source edge. A general straight-skeleton
                // decision is still required when that evidence is exhausted.
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            let component = match self_contacts {
                Classification::Decided(false) => {
                    curve_region_from_native_material_contour(raw_offset, policy)?
                }
                Classification::Decided(true) if component_expands => {
                    regularize_native_contour_with_curve_region(&raw_offset, policy)?
                }
                Classification::Decided(true) => {
                    // Remaining non-orthogonal contracting self-intersections
                    // need straight-skeleton pruning, not only winding
                    // regularization. Keep that gap explicit rather than
                    // retaining inverted collapse loops.
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            push_native_offset_component(
                *role,
                component,
                &mut material_components,
                &mut void_components,
            );
        }
        let edited =
            regularize_native_offset_regions(material_components, void_components, policy)?;
        Ok(Classification::Decided(edited))
    }

    fn exact_axis_aligned_algebraic_source_loops(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Arc<[Option<Arc<ExactAxisAlignedAlgebraicSourceLoop2>>]>>> {
        if let Some((cached_policy, certainty, source_loops)) =
            self.data.axis_aligned_algebraic_source_loops.get()
            && cached_policy == policy
        {
            if *certainty == CurveCertainty::Approximate512Consumed {
                policy.observe_approximate_512();
            }
            return Ok(Classification::Decided(Arc::clone(source_loops)));
        }
        let outcome = resolve_certified_value(
            policy,
            |attempt| -> CurveResult<
                Classification<Arc<[Option<Arc<ExactAxisAlignedAlgebraicSourceLoop2>>]>>,
            > {
                let mut source_loops = Vec::with_capacity(self.data.boundary_loops.len());
                for boundary_loop in &self.data.boundary_loops {
                    match exact_axis_aligned_algebraic_source_loop(boundary_loop, attempt)? {
                        Classification::Decided(source_loop) => {
                            source_loops.push(source_loop);
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Ok(Classification::Decided(Arc::from(source_loops)))
            },
        );
        let source_loops = match outcome.value? {
            Classification::Decided(source_loops) => source_loops,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let _ = self.data.axis_aligned_algebraic_source_loops.set((
            *policy,
            outcome.certainty,
            Arc::clone(&source_loops),
        ));
        Ok(Classification::Decided(source_loops))
    }

    fn offset_exact_general_raw(
        &self,
        distance: Real,
        corner_style: &OffsetCornerStyle2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Self>> {
        if is_zero(&distance, policy) == Some(true) {
            return Ok(Classification::Decided(self.clone()));
        }
        // Native line/arc contraction topology is a private specialization of
        // this authority.  It may decide exact wavefront collapse and neck
        // splitting early, but every component is composed through
        // `CurveRegion2::boolean_region_raw`; an inapplicable specialization
        // rejoins the general retained-carrier construction below.
        match self.try_offset_native_topology_fast_path(distance.clone(), corner_style, policy) {
            Ok(decided @ Classification::Decided(_)) => return Ok(decided),
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported)) => {}
            Err(ExactCurveError::Blocked(blocker))
                if blocker.reason() == UncertaintyReason::Unsupported => {}
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(error) => return Err(error),
        }
        let distance_positive = match real_sign(&distance, policy) {
            Some(RealSign::Positive) => true,
            Some(RealSign::Negative) => false,
            Some(RealSign::Zero) => return Ok(Classification::Decided(self.clone())),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let roles = match self
            .loop_roles_raw(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
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
        let fill_rules = self.loop_fill_rules().map_or_else(
            || vec![FillRule::EvenOdd; self.data.boundary_loops.len()],
            <[_]>::to_vec,
        );
        let axis_aligned_algebraic_source_loops = match self
            .exact_axis_aligned_algebraic_source_loops(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(source_loops) => source_loops,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if axis_aligned_algebraic_source_loops.len() != self.data.boundary_loops.len() {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology(
                    "cached algebraic offset sources are inconsistent with boundary loops".into(),
                ),
            ));
        }
        if roles.len() != self.data.boundary_loops.len()
            || filled_sides.len() != self.data.boundary_loops.len()
            || fill_rules.len() != self.data.boundary_loops.len()
        {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology(
                    "exact offset semantics are inconsistent with boundary loops".into(),
                ),
            ));
        }

        let certify_cardinal_convex_outward_topology = distance_positive
            && self.data.boundary_loops.len() == 1
            && roles[0] == CurveRegionLoopRole::Material
            && filled_sides[0]
            && self.has_certified_regularized_filled_left_topology()
            && matches!(corner_style, OffsetCornerStyle2::Round)
            && matches!(
                exact_cardinal_convex_filled_left_loop(&self.data.boundary_loops[0], policy)
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?,
                Classification::Decided(true)
            );

        let mut offset_loops = Vec::with_capacity(self.data.boundary_loops.len());
        let mut offset_roles = Vec::with_capacity(self.data.boundary_loops.len());
        let mut offset_fill_rules = Vec::with_capacity(self.data.boundary_loops.len());
        let mut offset_filled_sides = Vec::with_capacity(self.data.boundary_loops.len());
        let mut certified_regularized_filled_left_topology = false;
        for (loop_index, boundary_loop) in self.data.boundary_loops.iter().enumerate() {
            let signed_left_distance = if filled_sides[loop_index] {
                Real::zero() - &distance
            } else {
                distance.clone()
            };
            let component_contracts =
                (roles[loop_index] == CurveRegionLoopRole::Material) != distance_positive;
            match exact_axis_aligned_algebraic_offset_loop(
                axis_aligned_algebraic_source_loops[loop_index].as_deref(),
                &signed_left_distance,
                corner_style,
                component_contracts,
                self.data.boundary_loops.len() == 1
                    && roles[loop_index] == CurveRegionLoopRole::Material,
                fill_rules[loop_index],
                policy,
            )
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(ExactAxisAlignedAlgebraicOffsetLoop2::Boundary(
                    offset_loop,
                )) => {
                    offset_loops.push(offset_loop);
                    offset_roles.push(roles[loop_index]);
                    offset_fill_rules.push(fill_rules[loop_index]);
                    offset_filled_sides.push(filled_sides[loop_index]);
                    continue;
                }
                Classification::Decided(
                    ExactAxisAlignedAlgebraicOffsetLoop2::ErodedBoundaries(boundaries),
                ) => {
                    let source_role = roles[loop_index];
                    certified_regularized_filled_left_topology = self.data.boundary_loops.len()
                        == 1
                        && source_role == CurveRegionLoopRole::Material;
                    for (boundary, outer) in boundaries {
                        offset_loops.push(boundary);
                        offset_roles.push(if outer {
                            source_role
                        } else {
                            match source_role {
                                CurveRegionLoopRole::Material => CurveRegionLoopRole::Hole,
                                CurveRegionLoopRole::Hole => CurveRegionLoopRole::Material,
                            }
                        });
                        offset_fill_rules.push(fill_rules[loop_index]);
                        offset_filled_sides.push(source_role == CurveRegionLoopRole::Material);
                    }
                    continue;
                }
                Classification::Decided(ExactAxisAlignedAlgebraicOffsetLoop2::Removed) => {
                    continue;
                }
                Classification::Decided(ExactAxisAlignedAlgebraicOffsetLoop2::Inapplicable) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let source_fragments = boundary_loop.fragments();
            let mut spans = Vec::with_capacity(boundary_loop.len());
            let processing_start = match source_fragments
                .first()
                .and_then(RetainedParallelOffsetFragmentRef2::from_fragment)
            {
                Some(first)
                    if retained_parallel_represented_parameter(
                        &retained_parallel_traversal_start(first),
                    )
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
            let mut processed = 0;
            while processed < source_fragments.len() {
                let fragment_index = (processing_start + processed) % source_fragments.len();
                let fragment = &source_fragments[fragment_index];
                let mut consumed = 1;
                let offset = match fragment {
                    BezierSplitFragment2::Materialized { curve, .. } => {
                        exact_offset_span_from_materialized_curve(
                            curve,
                            &signed_left_distance,
                            policy,
                        )
                    }
                    BezierSplitFragment2::AnalyticParallel(_)
                    | BezierSplitFragment2::SelectedFiber(_) => {
                        match coalesced_retained_parallel_offset_run(
                            source_fragments,
                            fragment_index,
                            source_fragments.len() - processed,
                            policy,
                        )
                        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                        {
                            Classification::Decided(Some((coalesced, run_length))) => {
                                consumed = run_length;
                                exact_offset_span_from_analytic_parallel(
                                    &coalesced,
                                    &signed_left_distance,
                                    policy,
                                )
                            }
                            Classification::Decided(None) => {
                                exact_offset_span_from_retained_parallel_fragment(
                                    RetainedParallelOffsetFragmentRef2::from_fragment(fragment)
                                        .expect("the retained-parallel match arm owns its view"),
                                    &signed_left_distance,
                                    policy,
                                )
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    BezierSplitFragment2::AlgebraicChord(chord) => {
                        exact_offset_span_from_algebraic_chord(chord, &signed_left_distance, policy)
                    }
                    BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                        match coalesced_algebraic_circle_offset_run(
                            source_fragments,
                            fragment_index,
                            source_fragments.len() - processed,
                            policy,
                        )
                        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                        {
                            Some((coalesced, run_length)) => {
                                consumed = run_length;
                                exact_offset_span_from_algebraic_cusp_semicircle(
                                    &coalesced,
                                    &signed_left_distance,
                                    policy,
                                )
                            }
                            None => exact_offset_span_from_algebraic_cusp_semicircle(
                                fragment,
                                &signed_left_distance,
                                policy,
                            ),
                        }
                    }
                    BezierSplitFragment2::AlgebraicEndpointImages {
                        reversed,
                        start,
                        end,
                        source_curve,
                        ..
                    } => exact_offset_span_from_algebraic_endpoint_images(
                        *reversed,
                        start,
                        end,
                        source_curve,
                        &signed_left_distance,
                        policy,
                    ),
                    BezierSplitFragment2::Unresolved { .. } => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                };
                match offset
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                {
                    Classification::Decided(span) => spans.push(span),
                    Classification::Uncertain(reason) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-exact-offset-blocker",
                            "span",
                        );
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                processed += consumed;
            }
            if spans.is_empty() {
                return Err(curve_region_edit_error(
                    CurveOperation2::Offset,
                    CurveError::Topology("exact offset loop has no source spans".into()),
                ));
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
                    Classification::Decided(()) => {}
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
            let retain_convex_arrangement_sources =
                certify_cardinal_convex_outward_topology && loop_index == 0;
            let offset_loop = if retain_convex_arrangement_sources {
                let arrangement_sources = (0..fragments.len())
                    .map(|fragment_index| {
                        CurveRegionFragmentSource2::new(fragment_index, fragment_index, 0)
                    })
                    .collect();
                CurveRegionBoundaryLoop2::try_new_with_arrangement_sources(
                    fragments,
                    arrangement_sources,
                    policy,
                )
            } else {
                CurveRegionBoundaryLoop2::new(fragments, policy)
            }
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
            if retain_convex_arrangement_sources {
                // The outward parallel of a convex set is convex. The source
                // face walk already proves simplicity, while the cardinal
                // turn inventory above proves convexity without sampling.
                // Unique fragment sources preserve that proof for subsequent
                // offsets and Booleans without rebuilding a unary graph.
                certified_regularized_filled_left_topology = true;
            }
            offset_loops.push(offset_loop);
            offset_roles.push(roles[loop_index]);
            offset_fill_rules.push(fill_rules[loop_index]);
            offset_filled_sides.push(filled_sides[loop_index]);
        }

        let mut raw = Self::new(offset_loops)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
        {
            let data = raw.data_mut_for_construction();
            data.certified_loop_roles = Some(Arc::from(offset_roles));
            data.certified_loop_fill_rules = Some(Arc::from(offset_fill_rules));
            data.signed_loop_composition = true;
        }
        raw = raw
            .with_certified_filled_side_is_left(offset_filled_sides)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
        if certified_regularized_filled_left_topology {
            raw = raw
                .with_certified_regularized_filled_left_topology()
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
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

    /// Offsets arbitrary materialized curve families through certified exact-scalar segmentation.
    ///
    /// [`Self::offset`] first attempts the authoritative exact kernel with no
    /// loss. Only when that kernel reports `Unsupported` is each retained
    /// Bezier or rational span subdivided until its control hull certifies the
    /// requested source-curve chord error. The emitted vertices remain
    /// [`Real`] values, and the resulting line topology is offset and
    /// regularized by the exact native kernel. The evidence explicitly marks
    /// this as a lossy boundary: the certificate bounds source-to-chord error,
    /// not Hausdorff error of the final parallel curve.
    pub fn offset_with_certified_segmentation(
        &self,
        distance: Real,
        corner_style: &OffsetCornerStyle2,
        options: &BezierFlatteningOptions,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<CurveRegionSegmentedOffsetResult2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.offset_with_certified_segmentation_raw(distance, corner_style, options, attempt)
        })
    }

    fn offset_with_certified_segmentation_raw(
        &self,
        distance: Real,
        corner_style: &OffsetCornerStyle2,
        options: &BezierFlatteningOptions,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<CurveRegionSegmentedOffsetResult2>> {
        match self.offset_raw(distance.clone(), corner_style, policy)? {
            Classification::Decided(region) => {
                return Ok(Classification::Decided(CurveRegionSegmentedOffsetResult2 {
                    region,
                    evidence: CurveRegionSegmentedOffsetEvidence2 {
                        used_exact_authoritative_path: true,
                        max_source_chord_error: options.max_error().clone(),
                        loop_evidence: Vec::new(),
                        lossy_boundary: false,
                    },
                }));
            }
            Classification::Uncertain(UncertaintyReason::Unsupported) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let segmented = match self.segment_certified_raw(options, policy)? {
            Classification::Decided(segmented) => segmented,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (segmented_source, segmentation_evidence) = segmented.into_parts();
        let region = match segmented_source.offset_raw(distance, corner_style, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(CurveRegionSegmentedOffsetResult2 {
            region,
            evidence: CurveRegionSegmentedOffsetEvidence2 {
                used_exact_authoritative_path: false,
                max_source_chord_error: segmentation_evidence.max_source_chord_error,
                loop_evidence: segmentation_evidence.loop_evidence,
                lossy_boundary: segmentation_evidence.lossy_boundary,
            },
        }))
    }

    /// Offsets general smooth polynomial boundaries through certified analytic parallels.
    ///
    /// The method first preserves any result from the authoritative exact
    /// offset kernel. Otherwise it constructs exact PH or conservatively
    /// verified Blend2D parallels for every smooth boundary path, chordizes
    /// those *output* curves with a separate certificate, and regularizes the
    /// resulting line arrangement.
    /// The pre-regularization directed bound is
    /// `parallel_fit_error + output_chord_error`. Because regularization can
    /// remove raw branches, the result does not promote that into a Hausdorff
    /// claim for the final topology.
    /// Authored corners and unsupported curve families fall back to
    /// [`Self::offset_with_certified_segmentation`]; that fallback is identified
    /// explicitly and does not claim a final parallel Hausdorff bound.
    pub fn offset_with_certified_bezier_parallel(
        &self,
        distance: Real,
        corner_style: &OffsetCornerStyle2,
        parallel_options: &BezierParallelVerificationOptions,
        output_flattening: &BezierFlatteningOptions,
        fallback_source_flattening: &BezierFlatteningOptions,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<CurveRegionCertifiedParallelOffsetResult2>>>
    {
        resolve_certified_operation(policy, |attempt| {
            self.offset_with_certified_bezier_parallel_raw(
                distance,
                corner_style,
                parallel_options,
                output_flattening,
                fallback_source_flattening,
                attempt,
            )
        })
    }

    fn offset_with_certified_bezier_parallel_raw(
        &self,
        distance: Real,
        corner_style: &OffsetCornerStyle2,
        parallel_options: &BezierParallelVerificationOptions,
        output_flattening: &BezierFlatteningOptions,
        fallback_source_flattening: &BezierFlatteningOptions,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<CurveRegionCertifiedParallelOffsetResult2>> {
        match self.offset_raw(distance.clone(), corner_style, policy)? {
            Classification::Decided(region) => {
                return Ok(Classification::Decided(
                    CurveRegionCertifiedParallelOffsetResult2 {
                        region,
                        evidence: CurveRegionCertifiedParallelOffsetEvidence2 {
                            used_exact_authoritative_path: true,
                            used_certified_parallel_path: false,
                            used_segmented_source_fallback: false,
                            max_parallel_fit_error: Real::zero(),
                            max_output_chord_error: Real::zero(),
                            certified_pre_regularization_boundary_error: Some(Real::zero()),
                            final_boundary_hausdorff_certified: true,
                            loop_evidence: Vec::new(),
                            fallback_evidence: None,
                        },
                    },
                ));
            }
            Classification::Uncertain(UncertaintyReason::Unsupported) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let paths =
            match self.materialized_boundary_paths_for_edit(CurveOperation2::Offset, policy)? {
                Classification::Decided(paths) => paths,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let roles = match self
            .loop_roles_raw(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
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
        let fill_rules = self
            .loop_fill_rules()
            .map_or_else(|| vec![FillRule::EvenOdd; paths.len()], <[_]>::to_vec);
        if paths.len() != roles.len()
            || paths.len() != filled_sides.len()
            || paths.len() != fill_rules.len()
        {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology(
                    "certified parallel semantics do not match boundary loops".into(),
                ),
            ));
        }

        let mut parallel_paths = Vec::with_capacity(paths.len());
        let mut loop_evidence = Vec::with_capacity(paths.len());
        let mut needs_fallback = false;
        for (((path, role), filled_side_is_left), fill_rule) in paths
            .iter()
            .zip(roles.iter())
            .zip(filled_sides.iter())
            .zip(fill_rules.iter())
        {
            let signed_left_distance = if *filled_side_is_left {
                Real::zero() - &distance
            } else {
                distance.clone()
            };
            let parallel = match path.approximate_parallel_blend2d_certified(
                signed_left_distance.clone(),
                parallel_options,
                policy,
            )? {
                Classification::Decided(parallel) => parallel,
                Classification::Uncertain(
                    UncertaintyReason::Unsupported | UncertaintyReason::Boundary,
                ) => {
                    needs_fallback = true;
                    break;
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            loop_evidence.push(CurveRegionCertifiedParallelLoopEvidence2 {
                role: *role,
                fill_rule: *fill_rule,
                signed_left_distance,
                source_curve_count: parallel.source_curve_count(),
                output_curve_count: parallel.output_curve_count(),
                exact_source_curve_count: parallel.exact_source_curve_count(),
                approximated_source_curve_count: parallel.approximated_source_curve_count(),
                verification_leaf_count: parallel.verification_leaf_count(),
            });
            parallel_paths.push(parallel.into_path());
        }

        if needs_fallback {
            return wrap_segmented_parallel_fallback(
                self.offset_with_certified_segmentation_raw(
                    distance,
                    corner_style,
                    fallback_source_flattening,
                    policy,
                )?,
                parallel_options.max_error(),
                output_flattening.max_error(),
            );
        }

        let parallel_region = Self::try_from_boundary_paths_with_loop_semantics_raw(
            &parallel_paths,
            roles.as_ref(),
            &fill_rules,
            policy,
            Some(filled_sides.as_ref().to_vec()),
        )?;
        let segmented = match parallel_region.segment_certified_raw(output_flattening, policy)? {
            Classification::Decided(segmented) => segmented,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let segmented_region = segmented.region();
        let native = segmented_region
            .data
            .line_image_region
            .certified()
            .and_then(Option::as_ref)
            .expect("certified segmentation always installs native line topology");
        let mut material_components = Vec::new();
        let mut void_components = Vec::new();
        for contour in native.material_contours() {
            material_components.push(regularize_native_contour_with_curve_region(
                contour, policy,
            )?);
        }
        for contour in native.hole_contours() {
            void_components.push(regularize_native_contour_with_curve_region(
                contour, policy,
            )?);
        }
        let regularized =
            regularize_native_offset_regions(material_components, void_components, policy)?;
        let certified_pre_regularization_boundary_error =
            parallel_options.max_error() + output_flattening.max_error();
        Ok(Classification::Decided(
            CurveRegionCertifiedParallelOffsetResult2 {
                region: regularized,
                evidence: CurveRegionCertifiedParallelOffsetEvidence2 {
                    used_exact_authoritative_path: false,
                    used_certified_parallel_path: true,
                    used_segmented_source_fallback: false,
                    max_parallel_fit_error: parallel_options.max_error().clone(),
                    max_output_chord_error: output_flattening.max_error().clone(),
                    certified_pre_regularization_boundary_error: Some(
                        certified_pre_regularization_boundary_error,
                    ),
                    final_boundary_hausdorff_certified: false,
                    loop_evidence,
                    fallback_evidence: None,
                },
            },
        ))
    }

    fn region_from_line_role_evidence(
        &self,
        evidence: &CurveRegionLineRoleEvidence2,
    ) -> CurveResult<LineArcRegion2> {
        let roles = self
            .data
            .certified_loop_roles
            .as_deref()
            .unwrap_or_else(|| evidence.roles());
        self.region_from_line_contours(evidence.contours(), roles)
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
                Classification::Decided(line_loop) => contours.push(line_loop.contour),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        self.region_from_line_contours(&contours, roles)
            .map(Classification::Decided)
    }

    fn materialized_native_line_arc_region(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<LineArcRegion2>> {
        let Some(native_loops) = self.native_boundary_loops() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let mut contours = Vec::with_capacity(native_loops.len());
        for (loop_index, boundary_loop) in native_loops.iter().enumerate() {
            let fill_rule = self
                .data
                .certified_loop_fill_rules
                .as_deref()
                .and_then(|rules| rules.get(loop_index))
                .copied()
                .unwrap_or(FillRule::NonZero);
            match materialized_native_loop_to_contour(boundary_loop, fill_rule, policy)? {
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

    /// Classifies a point against the exact retained region.
    ///
    /// Native polynomial and rational boundary fragments use certified ray
    /// incidence directly. Exact line-image algebraic carriers are lowered once
    /// to a clone-shared native line region. Nonlinear algebraic carriers with
    /// retained source curves filter exact source-curve incidence to their
    /// represented parameter ranges. A non-line carrier without source-curve
    /// provenance remains explicit `Unsupported` uncertainty.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<RegionPointLocation>>> {
        resolve_certified_operation(policy, |attempt| self.classify_point_raw(point, attempt))
    }

    /// Classifies a batch of exact points against the unified region.
    ///
    /// Native line/arc topology builds its query indexes once. Retained curved
    /// topology reuses the same authoritative scalar classifier for every
    /// point, preserving explicit uncertainty and the operation-wide policy
    /// terminal.
    pub fn classify_points(
        &self,
        points: &[Point2],
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Vec<Classification<RegionPointLocation>>>> {
        resolve_certified_operation(policy, |attempt| {
            if !self.data.signed_loop_composition
                && let Classification::Decided(native) = self.native_line_arc_region(attempt)?
            {
                return Ok(native.classify_points(points, attempt));
            }
            points
                .iter()
                .map(|point| self.classify_point_raw(point, attempt))
                .collect()
        })
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
                    Classification::Decided(native.structural_facts(attempt))
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            })
        })
    }

    /// Classifies an exact algebraic point against the retained region.
    ///
    /// The point remains in its defining local algebraic field. Boundary
    /// incidence and ray winding therefore consume the same `STRICT` or
    /// `APPROXIMATE_512` predicate policy as the rest of the curve kernel.
    pub fn classify_algebraic_point(
        &self,
        point: &RationalBezierAlgebraicPointImage2,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<RegionPointLocation>>> {
        resolve_certified_operation(policy, |attempt| {
            self.classify_algebraic_point_raw(point, attempt)
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
        if !self.data.signed_loop_composition {
            match self.native_line_arc_region(policy)? {
                Classification::Decided(region) => {
                    return Ok(region.classify_point(point, policy));
                }
                Classification::Uncertain(UncertaintyReason::Unsupported) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let Some(native_loops) = self.native_boundary_loops() else {
            return classify_point_against_retained_loops(
                &self.data.boundary_loops,
                self.retained_rational_evaluators()?,
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

    pub(crate) fn classify_point_from_boundary_side_ray(
        &self,
        point: &Point2,
        direction_x: Real,
        direction_y: Real,
        direction_is_certified_nonzero: bool,
        source_crossing_direction: BezierLineCrossingDirection,
        source_loop_index: usize,
        source_fragment_index: usize,
        source_parameter: Option<&CurveRegionParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RegionPointLocation>> {
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

        let endpoint = Point2::new(point.x() + &direction_x, point.y() + &direction_y);
        let ray = BezierRay2 {
            line: LineSeg2::try_new(point.clone(), endpoint)?,
            direction_x,
            direction_y,
        };
        let source_tangent_contacts = retained_circle_tangent_contacts(
            &self.data.boundary_loops[source_loop_index].fragments()[source_fragment_index],
        );
        let mut inside = false;
        let mut signed_depth = 0_i32;
        for (loop_index, boundary_loop) in self.data.boundary_loops.iter().enumerate() {
            let fill_rule = self
                .data
                .certified_loop_fill_rules
                .as_ref()
                .map_or(FillRule::EvenOdd, |rules| rules[loop_index]);
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
                fill_rule,
                skipped_origin,
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
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
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

    pub(crate) fn classify_algebraic_point_from_boundary_side_ray(
        &self,
        point: &RationalBezierAlgebraicPointImage2,
        direction_x: Real,
        direction_y: Real,
        source_loop_index: usize,
        source_fragment_index: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RegionPointLocation>> {
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
        let point = match point.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let side_x = -direction_y.clone();
        let side_y = direction_x.clone();
        let mut inside = false;
        let mut signed_depth = 0_i32;
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
                policy,
            )? {
                Classification::Decided(winding) => winding,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let fill_rule = self
                .data
                .certified_loop_fill_rules
                .as_ref()
                .map_or(FillRule::EvenOdd, |rules| rules[loop_index]);
            if winding_location(winding, fill_rule) == ContourPointLocation::Inside {
                if let Some(roles) = &self.data.certified_loop_roles {
                    signed_depth += match roles[loop_index] {
                        CurveRegionLoopRole::Material => 1,
                        CurveRegionLoopRole::Hole => -1,
                    };
                } else {
                    inside = !inside;
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

    /// Returns signed material-minus-hole containment depth for a non-boundary point.
    ///
    /// Explicit roles are authoritative. Otherwise the exact curved nesting
    /// classifier derives one role per loop before depth accumulation. Each
    /// loop's own fill rule controls whether it contributes at the query point.
    /// Boundary points return `Uncertain(Boundary)` rather than an arbitrary
    /// integer.
    pub fn signed_depth(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<i32>>> {
        resolve_certified_operation(policy, |attempt| self.signed_depth_raw(point, attempt))
    }

    pub(crate) fn signed_depth_raw(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        if !self.data.signed_loop_composition {
            match self.native_line_arc_region(policy)? {
                Classification::Decided(region) => {
                    return Ok(region.signed_depth(point, policy));
                }
                Classification::Uncertain(_) => {}
            }
        }
        self.signed_depth_from_boundaries(point, policy)
    }

    fn signed_depth_from_boundaries(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        let roles = match self.loop_roles_raw(policy)? {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if roles.len() != self.data.boundary_loops.len() {
            return Err(CurveError::Topology(
                "curve-region signed-depth roles are inconsistent with boundary loops".into(),
            ));
        }
        if self
            .data
            .certified_loop_fill_rules
            .as_ref()
            .is_some_and(|rules| rules.len() != self.data.boundary_loops.len())
        {
            return Err(CurveError::Topology(
                "curve-region signed-depth fill rules are inconsistent with boundary loops".into(),
            ));
        }

        let mut depth = 0_i32;
        if let Some(native_loops) = self.native_boundary_loops() {
            let native_bounds = self.native_boundary_bounds(policy);
            for (index, (boundary_loop, role)) in native_loops.iter().zip(&roles).enumerate() {
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
                        depth += curve_region_role_depth(*role);
                    }
                    Classification::Decided(ContourPointLocation::Outside) => {}
                    Classification::Decided(ContourPointLocation::Boundary) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            return Ok(Classification::Decided(depth));
        }

        let evaluators = self.retained_rational_evaluators()?;
        if evaluators.len() != self.data.boundary_loops.len() {
            return Err(CurveError::Topology(
                "curve-region signed-depth evaluator cache is inconsistent with boundary loops"
                    .into(),
            ));
        }
        for (index, ((boundary_loop, evaluators), role)) in self
            .data
            .boundary_loops
            .iter()
            .zip(evaluators)
            .zip(&roles)
            .enumerate()
        {
            let fill_rule = self
                .data
                .certified_loop_fill_rules
                .as_ref()
                .map_or(FillRule::EvenOdd, |rules| rules[index]);
            match classify_point_against_retained_loop_with_fill_rule(
                boundary_loop,
                evaluators,
                point,
                fill_rule,
                policy,
            )? {
                Classification::Decided(ContourPointLocation::Inside) => {
                    depth += curve_region_role_depth(*role);
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(depth))
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
            let magnitude = match if self.has_certified_regularized_filled_left_topology() {
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

    fn retained_rational_evaluators(&self) -> CurveResult<&[Vec<Option<RationalBezier2>>]> {
        match self.data.retained_rational_evaluators.get_or_init(|| {
            self.data
                .boundary_loops
                .iter()
                .map(|boundary_loop| {
                    boundary_loop
                        .fragments()
                        .iter()
                        .map(|fragment| match fragment {
                            BezierSplitFragment2::AlgebraicEndpointImages {
                                source_curve: Some(source_curve),
                                ..
                            } => rationalize_retained_subcurve(source_curve).map(Some),
                            _ => Ok(None),
                        })
                        .collect()
                })
                .collect()
        }) {
            Ok(evaluators) => Ok(evaluators),
            Err(error) => Err(error.clone()),
        }
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
            BezierSplitFragment2::AlgebraicChord(chord) => {
                let Some(line) = chord.exact_line() else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                curves.push(Curve2::from(line));
            }
            _ => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
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
            CurveIntersectionPairBlockerKind2::IncompleteReplay { .. } => {
                UncertaintyReason::Predicate
            }
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
    let (Some(first), Some(second)) = (
        contact.contact().first().exact_curve_parameter(),
        contact.contact().second().exact_curve_parameter(),
    ) else {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    };
    let mut uncertain = false;
    for (expected_first, expected_second) in consecutive.into_iter().chain(closing) {
        match (
            compare_reals(&first, expected_first, policy),
            compare_reals(&second, expected_second, policy),
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
        BezierSplitFragment2::AlgebraicEndpointImages {
            reversed,
            start,
            end,
            source_curve: Some(source),
            ..
        } => {
            let source =
                transform_region_subcurve(source, m00, m01, m10, m11, tx, ty, similarity.as_ref())?;
            Ok(BezierSplitFragment2::AlgebraicEndpointImages {
                reversed: *reversed,
                start: start.clone(),
                end: end.clone(),
                start_image: transform_region_endpoint_image(start, &source, policy)?,
                end_image: transform_region_endpoint_image(end, &source, policy)?,
                source_curve: Some(source),
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
        BezierSplitFragment2::AlgebraicChord(chord) => match chord
            .transform_affine(m00, m01, m10, m11, tx, ty, policy)
            .map_err(affine_region_error)?
        {
            Classification::Decided(chord) => Ok(BezierSplitFragment2::AlgebraicChord(chord)),
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
        BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: None, ..
        }
        | BezierSplitFragment2::Unresolved { .. } => Err(ExactCurveError::blocked(
            CurveOperation2::Transformation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        )),
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let transform = retained_similarity()?;
            let source = match fragment.source() {
                BezierSelectedFiberSource2::Rational(curve) => {
                    BezierSelectedFiberSource2::Rational(
                        RationalBezier2::try_new(
                            curve
                                .control_points()
                                .iter()
                                .map(|point| transform.transform_point(point))
                                .collect(),
                            curve.weights().to_vec(),
                        )
                        .map_err(affine_region_error)?,
                    )
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
            let transformed_start = RationalBezierIntersectionPointEvidence2::Similarity(
                crate::BezierSimilarityPoint2::new(source_start.clone(), transform.clone(), policy),
            );
            let transformed_end = RationalBezierIntersectionPointEvidence2::Similarity(
                crate::BezierSimilarityPoint2::new(source_end.clone(), transform.clone(), policy),
            );
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
) -> ExactCurveResult<Option<BezierAlgebraicEndpointImage2>> {
    match parameter {
        BezierParameter2::Exact(_) => Ok(None),
        BezierParameter2::Algebraic(parameter) => {
            BezierAlgebraicEndpointImage2::from_source_curve_first_order(source, parameter, policy)
                .map(Some)
                .map_err(affine_region_error)
        }
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
            RationalBezier2::try_new(
                curve.control_points().iter().map(point).collect(),
                curve.weights().to_vec(),
            )
            .map_err(affine_region_error)?,
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

struct RetainedLineLoopContour {
    contour: Contour2,
    materialized_fragment_count: usize,
    algebraic_fragment_count: usize,
}

fn retained_loop_arrangement_sources(
    boundary_loops: &[CurveRegionBoundaryLoop2],
) -> Vec<Option<Vec<CurveRegionFragmentSource2>>> {
    boundary_loops
        .iter()
        .map(|boundary_loop| boundary_loop.arrangement_sources().map(<[_]>::to_vec))
        .collect()
}

fn retained_loop_fragment_counts(boundary_loops: &[CurveRegionBoundaryLoop2]) -> Vec<usize> {
    boundary_loops
        .iter()
        .map(CurveRegionBoundaryLoop2::len)
        .collect()
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

fn validate_loop_fragment_counts(
    loop_count: usize,
    loop_fragment_counts: &[usize],
) -> CurveResult<()> {
    validate_evidence_length(
        loop_count,
        "loop fragment count",
        loop_fragment_counts.len(),
    )?;
    if loop_fragment_counts.contains(&0) {
        return Err(CurveError::Topology(
            "retained role evidence loop fragment counts must be nonzero".into(),
        ));
    }
    Ok(())
}

fn validate_loop_arrangement_sources(
    loop_count: usize,
    loop_arrangement_sources: &[Option<Vec<CurveRegionFragmentSource2>>],
) -> CurveResult<()> {
    validate_evidence_length(
        loop_count,
        "loop arrangement source",
        loop_arrangement_sources.len(),
    )?;
    if loop_arrangement_sources.iter().flatten().any(Vec::is_empty) {
        return Err(CurveError::Topology(
            "retained role evidence present loop arrangement sources must be nonempty".into(),
        ));
    }
    let indices = loop_arrangement_sources
        .iter()
        .filter_map(Option::as_ref)
        .flat_map(|sources| {
            sources
                .iter()
                .map(|source| source.arrangement_fragment_index())
        })
        .collect::<Vec<_>>();
    validate_unique_arrangement_source_indices(
        indices,
        "retained role evidence loop arrangement sources must not reuse arrangement fragments",
    )
}

fn validate_counted_loop_arrangement_source_counts(
    loop_fragment_counts: Option<&[usize]>,
    loop_arrangement_sources: &[Option<Vec<CurveRegionFragmentSource2>>],
) -> CurveResult<()> {
    let Some(loop_fragment_counts) = loop_fragment_counts else {
        if loop_arrangement_sources.iter().any(Option::is_some) {
            return Err(CurveError::Topology(
                "retained role evidence present loop arrangement sources require loop fragment count evidence"
                    .into(),
            ));
        }
        return Ok(());
    };

    for (fragment_count, sources) in loop_fragment_counts.iter().zip(loop_arrangement_sources) {
        if let Some(sources) = sources
            && sources.len() != *fragment_count
        {
            return Err(CurveError::Topology(
                "retained role evidence loop source count does not match loop fragment count"
                    .into(),
            ));
        }
    }
    Ok(())
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

fn validate_evidence_length(
    loop_count: usize,
    evidence_name: &str,
    evidence_count: usize,
) -> CurveResult<()> {
    if loop_count == 0 {
        return Err(CurveError::Topology(
            "retained role evidence must carry at least one loop".into(),
        ));
    }
    if loop_count != evidence_count {
        return Err(CurveError::Topology(format!(
            "retained role evidence {evidence_name} count does not match loop count"
        )));
    }
    Ok(())
}

fn validate_nesting_depth_roles(
    roles: &[CurveRegionLoopRole],
    nesting_depths: &[usize],
) -> CurveResult<()> {
    for (role, depth) in roles.iter().zip(nesting_depths) {
        let expected = if depth.is_multiple_of(2) {
            CurveRegionLoopRole::Material
        } else {
            CurveRegionLoopRole::Hole
        };
        if *role != expected {
            return Err(CurveError::Topology(
                "retained nesting role evidence role does not match certified nesting depth".into(),
            ));
        }
    }
    Ok(())
}

fn validate_signed_area_roles(
    roles: &[CurveRegionLoopRole],
    signed_areas: &[Real],
    policy: &CurveContext,
) -> CurveResult<()> {
    for (role, signed_area) in roles.iter().zip(signed_areas) {
        let expected = match real_sign(signed_area, policy) {
            Some(RealSign::Negative) => CurveRegionLoopRole::Material,
            Some(RealSign::Positive) => CurveRegionLoopRole::Hole,
            Some(RealSign::Zero) | None => {
                return Err(CurveError::Topology(
                    "retained signed-area role evidence must carry certified nonzero area evidence"
                        .into(),
                ));
            }
        };
        if *role != expected {
            return Err(CurveError::Topology(
                "retained signed-area role evidence role does not match signed-area evidence"
                    .into(),
            ));
        }
    }
    Ok(())
}

fn validate_nonzero_signed_area_evidence(
    signed_areas: &[Real],
    policy: &CurveContext,
) -> CurveResult<()> {
    for signed_area in signed_areas {
        match real_sign(signed_area, policy) {
            Some(RealSign::Positive | RealSign::Negative) => {}
            Some(RealSign::Zero) | None => {
                return Err(CurveError::Topology(
                    "retained curved nesting role evidence must carry certified nonzero signed-area evidence"
                        .into(),
                ));
            }
        }
    }
    Ok(())
}

fn validate_line_role_evidence_fragment_counts(
    materialized_fragment_count: usize,
    algebraic_fragment_count: usize,
    contours: &[Contour2],
) -> CurveResult<()> {
    let source_fragment_count = materialized_fragment_count
        .checked_add(algebraic_fragment_count)
        .ok_or_else(|| {
            CurveError::Topology(
                "retained line role evidence source fragment count overflowed".into(),
            )
        })?;
    let contour_fragment_count = contours
        .iter()
        .try_fold(0_usize, |count, contour| count.checked_add(contour.len()))
        .ok_or_else(|| {
            CurveError::Topology(
                "retained line role evidence contour fragment count overflowed".into(),
            )
        })?;
    if source_fragment_count != contour_fragment_count {
        return Err(CurveError::Topology(
            "retained line role evidence source fragment count does not match line contour evidence"
                .into(),
        ));
    }
    Ok(())
}

fn validate_line_loop_arrangement_source_counts(
    contours: &[Contour2],
    loop_arrangement_sources: &[Option<Vec<CurveRegionFragmentSource2>>],
) -> CurveResult<()> {
    for (contour, sources) in contours.iter().zip(loop_arrangement_sources) {
        if let Some(sources) = sources
            && sources.len() != contour.len()
        {
            return Err(CurveError::Topology(
                "retained line role evidence loop source count does not match contour fragment count"
                    .into(),
            ));
        }
    }
    Ok(())
}

fn retained_line_loop_to_contour(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<RetainedLineLoopContour>> {
    let mut segments = Vec::with_capacity(boundary_loop.fragments().len());
    let mut materialized_fragment_count = 0_usize;
    let mut algebraic_fragment_count = 0_usize;
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
        match endpoints.source {
            RetainedLineFragmentSource::MaterializedFit => materialized_fragment_count += 1,
            RetainedLineFragmentSource::AlgebraicEndpoints => algebraic_fragment_count += 1,
        }
        let (start, end) = endpoints.points;
        segments.push(Segment2::Line(LineSeg2::try_new(start, end)?));
    }
    if let Some(reason) = blocker {
        return Ok(Classification::Uncertain(reason));
    }
    Contour2::try_new(segments).map(|contour| {
        Classification::Decided(RetainedLineLoopContour {
            contour,
            materialized_fragment_count,
            algebraic_fragment_count,
        })
    })
}

struct RetainedLineFragmentEndpoints {
    points: (Point2, Point2),
    source: RetainedLineFragmentSource,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RetainedLineFragmentSource {
    MaterializedFit,
    AlgebraicEndpoints,
}

/// Returns exact line-segment endpoints for a retained line-image fragment.
///
/// Materialized fragments must carry a certified exact endpoint line-image
/// fit. Algebraic endpoint-image fragments are accepted
/// only when the endpoint point evidence has exact rational witnesses, or when
/// an exact boundary parameter can be replayed against the retained source
/// curve. This follows the exactness model's retained-object discipline: algebraic endpoints
/// become line-contour topology only through exact construction evidence, not
/// by sampling isolating intervals. The native fit certificate proves every
/// control point lies on the endpoint segment, preserving the exact
/// object/predicate split described by the exactness model while allowing non-affine
/// parameterizations whose image is still exactly one line segment.
fn retained_line_fragment_endpoints(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<RetainedLineFragmentEndpoints>> {
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
            Ok(Classification::Decided(RetainedLineFragmentEndpoints {
                points: (fit.line().start().clone(), fit.line().end().clone()),
                source: RetainedLineFragmentSource::MaterializedFit,
            }))
        }
        BezierSplitFragment2::AlgebraicEndpointImages {
            reversed,
            start,
            end,
            source_curve,
            start_image,
            end_image,
        } => {
            if let Some(source_curve) = source_curve {
                match subcurve_fit_exact_line_image(source_curve, policy)? {
                    Classification::Decided(BezierLineImageFitRelation::Fit(_)) => {}
                    Classification::Decided(BezierLineImageFitRelation::NotLine) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
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
            Ok(Classification::Decided(RetainedLineFragmentEndpoints {
                points,
                source: RetainedLineFragmentSource::AlgebraicEndpoints,
            }))
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
            let Some((start_parameter, end_parameter)) = fragment.range().exact_endpoints() else {
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
            Ok(Classification::Decided(RetainedLineFragmentEndpoints {
                points: if fragment.is_reversed() {
                    (end, start)
                } else {
                    (start, end)
                },
                source: RetainedLineFragmentSource::AlgebraicEndpoints,
            }))
        }
        BezierSplitFragment2::AlgebraicChord(chord) => {
            let Some(line) = chord.exact_line() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            Ok(Classification::Decided(RetainedLineFragmentEndpoints {
                points: (line.start().clone(), line.end().clone()),
                source: RetainedLineFragmentSource::AlgebraicEndpoints,
            }))
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        BezierSplitFragment2::SelectedFiber(_) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        BezierSplitFragment2::Unresolved { .. } => {
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        }
    }
}

pub(crate) fn retained_line_fragment_segment(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<LineSeg2>> {
    let endpoints = match retained_line_fragment_endpoints(fragment, policy)? {
        Classification::Decided(endpoints) => endpoints.points,
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
    source_curve: &Option<BezierSubcurve2>,
    policy: &CurveContext,
) -> Classification<Point2> {
    match parameter {
        BezierParameter2::Exact(value) => {
            let Some(source_curve) = source_curve else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            subcurve_point_at(source_curve, value.clone(), policy)
        }
        BezierParameter2::Algebraic(_) => {
            let Some(image) = image else {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            };
            match exact_rational_point_from_image(image.point(), Some(policy)) {
                Some(point) => Classification::Decided(point),
                None => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
    }
}

fn exact_rational_point_from_image(
    point: &BezierEndpointPointImage2,
    resolution_policy: Option<&CurveContext>,
) -> Option<Point2> {
    match point {
        BezierEndpointPointImage2::Polynomial(point) => Some(Point2::new(
            point
                .x()?
                .representation()?
                .exact_rational_witness()?
                .clone(),
            point
                .y()?
                .representation()?
                .exact_rational_witness()?
                .clone(),
        )),
        BezierEndpointPointImage2::Rational(point) => {
            let policy = resolution_policy.unwrap_or(&CurveContext::STRICT);
            point.exact_rational_point(policy)
        }
    }
}

struct RetainedLoopRoleDecision {
    roles: Vec<CurveRegionLoopRole>,
    nesting_depths: Vec<usize>,
}

fn retained_line_loop_roles(
    contours: &[Contour2],
    policy: &CurveContext,
) -> CurveResult<Classification<RetainedLoopRoleDecision>> {
    let mut roles = Vec::with_capacity(contours.len());
    let mut nesting_depths = Vec::with_capacity(contours.len());
    for (candidate_index, candidate) in contours.iter().enumerate() {
        let sample = candidate
            .segments()
            .first()
            .ok_or(crate::CurveError::EmptyCurveString)?
            .start();
        let mut depth = 0_usize;
        for (container_index, container) in contours.iter().enumerate() {
            if candidate_index == container_index {
                continue;
            }
            match container.classify_point(sample, policy) {
                Classification::Decided(ContourPointLocation::Inside) => depth += 1,
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        nesting_depths.push(depth);
        roles.push(if depth.is_multiple_of(2) {
            CurveRegionLoopRole::Material
        } else {
            CurveRegionLoopRole::Hole
        });
    }
    Ok(Classification::Decided(RetainedLoopRoleDecision {
        roles,
        nesting_depths,
    }))
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

fn materialized_native_loop_to_contour(
    boundary_loop: &BezierBoundaryLoop2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<Contour2>> {
    let mut segments = Vec::with_capacity(boundary_loop.len());
    for curve in boundary_loop.fragments() {
        match materialized_native_subcurve_segment(curve, policy)? {
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

fn retained_loop_sample_point(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<Point2>> {
    let Some(fragment) = boundary_loop.fragments().first() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            let half = match Real::one() / Real::from(2_i8) {
                Ok(half) => half,
                Err(_) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
            };
            Ok(subcurve_point_at(curve, half, policy))
        }
        BezierSplitFragment2::AlgebraicEndpointImages {
            start,
            end,
            source_curve: Some(source_curve),
            ..
        } => {
            let parameter = match start.strict_rational_between(end, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(subcurve_point_at(source_curve, parameter, policy))
        }
        BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: None, ..
        } => {
            let endpoint = retained_fragment_endpoint_evidence(fragment, true, policy)?;
            Ok(endpoint.point.map_or(
                Classification::Uncertain(UncertaintyReason::Boundary),
                Classification::Decided,
            ))
        }
        BezierSplitFragment2::AnalyticParallel(fragment) => fragment.representative_point(policy),
        BezierSplitFragment2::SelectedFiber(fragment) => fragment.representative_point(policy),
        BezierSplitFragment2::AlgebraicChord(_) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        BezierSplitFragment2::Unresolved { .. } => {
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        }
    }
}

fn subcurve_control_hull_contains_point(
    curve: &BezierSubcurve2,
    point: &Point2,
    policy: &CurveContext,
) -> Classification<bool> {
    let bounds = match curve {
        BezierSubcurve2::Quadratic(curve) => Aabb2::from_points(curve.control_points(), policy),
        BezierSubcurve2::Cubic(curve) => Aabb2::from_points(curve.control_points(), policy),
        BezierSubcurve2::RationalQuadratic(curve) => {
            if curve.common_nonzero_weight_sign(policy).is_none() {
                return Classification::Uncertain(UncertaintyReason::RealSign);
            }
            Aabb2::from_points(curve.control_points(), policy)
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

fn classify_point_against_native_loop(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    if let Classification::Decided(bounds) = native_loop_bounds(boundary_loop, policy)
        && let Classification::Decided(false) = bounds.contains_point(point, policy)
    {
        return Ok(Classification::Decided(ContourPointLocation::Outside));
    }
    classify_point_against_native_loop_after_bounds(boundary_loop, point, policy)
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
    retained_range: Option<CurveRegionParameterRange2>,
    reversed: bool,
    endpoints: [RationalBezierIntersectionPointEvidence2; 2],
}

enum AlgebraicRayRetainedFragment2 {
    Rational(AlgebraicRayRationalFragment2),
    AnalyticParallel(crate::bezier_offset::BezierParallelAlgebraicRay2),
    AlgebraicChord(crate::bezier_offset::BezierAlgebraicChordAlgebraicRay2),
    AlgebraicCusp(crate::bezier_offset::BezierAlgebraicCuspSemicircleAlgebraicRay2),
}

#[derive(Default)]
struct AlgebraicRaySignHull2 {
    negative: bool,
    zero: bool,
    positive: bool,
    first_nonzero: Option<RealSign>,
    last_nonzero: Option<RealSign>,
}

impl AlgebraicRaySignHull2 {
    fn include(&mut self, sign: RealSign) {
        match sign {
            RealSign::Negative => self.negative = true,
            RealSign::Zero => {
                self.zero = true;
                return;
            }
            RealSign::Positive => self.positive = true,
        }
        self.first_nonzero.get_or_insert(sign);
        self.last_nonzero = Some(sign);
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
                    fragment.contains_point(point, policy)?
                }
                AlgebraicRayRetainedFragment2::AlgebraicChord(fragment) => {
                    fragment.contains_point(point, policy)?
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
                let evaluator = match chord.algebraic_ray_evaluator(policy)? {
                    Classification::Decided(evaluator) => evaluator,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                fragments.push(AlgebraicRayRetainedFragment2::AlgebraicChord(evaluator));
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
            AlgebraicRayRetainedFragment2::Rational(fragment) => {
                match algebraic_ray_rational_fragment_endpoint_side_signs(
                    fragment, point, side_x, side_y, policy,
                )? {
                    Classification::Decided(signs)
                        if signs.into_iter().all(|sign| sign != RealSign::Zero) => {}
                    Classification::Decided(_) => return Ok(Classification::Decided(false)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
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
                match fragment.endpoint_side_signs(point, side_x, side_y, policy)? {
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
    policy: &CurveContext,
) -> CurveResult<Classification<i32>> {
    let skipped_support = if let Some(fragment_index) = skipped_fragment {
        match fragments.get(fragment_index) {
            Some(AlgebraicRayRetainedFragment2::AlgebraicCusp(fragment)) => Some(fragment),
            Some(AlgebraicRayRetainedFragment2::AlgebraicChord(_)) => None,
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
    } else {
        None
    };
    let mut winding = 0_i32;
    for (fragment_index, fragment) in fragments.iter().enumerate() {
        // The side-ray origin lies strictly inside its source cusp subarc.
        // A line through a circle point has only that point and its antipode;
        // the antipode is outside the same open semicircle. Omitting this one
        // certified source fragment therefore removes exactly the origin
        // contact and no forward crossing.
        if skipped_fragment == Some(fragment_index) {
            continue;
        }
        let delta = match fragment {
            AlgebraicRayRetainedFragment2::Rational(fragment) => {
                algebraic_point_rational_curve_ray_winding(
                    fragment,
                    point,
                    direction_x,
                    direction_y,
                    policy,
                )?
            }
            AlgebraicRayRetainedFragment2::AnalyticParallel(fragment) => {
                fragment.forward_ray_winding_delta(point, direction_x, direction_y, policy)?
            }
            AlgebraicRayRetainedFragment2::AlgebraicChord(fragment) => {
                fragment.forward_ray_winding_delta(point, direction_x, direction_y, policy)?
            }
            AlgebraicRayRetainedFragment2::AlgebraicCusp(fragment) => {
                let point_on_supporting_circle = skipped_support
                    .is_some_and(|source| fragment.has_same_structural_support(source));
                fragment.forward_ray_winding_delta(
                    point,
                    direction_x,
                    direction_y,
                    point_on_supporting_circle,
                    policy,
                )?
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
) -> CurveResult<[RationalBezierIntersectionPointEvidence2; 2]> {
    let endpoint = |start_endpoint| -> CurveResult<_> {
        let evidence = retained_fragment_endpoint_evidence(fragment, start_endpoint, policy)?;
        evidence
            .retained_point
            .or_else(|| {
                evidence
                    .point
                    .map(RationalBezierIntersectionPointEvidence2::Exact)
            })
            .ok_or_else(|| {
                CurveError::Topology(
                    "a retained algebraic-ray fragment lost exact endpoint evidence".into(),
                )
            })
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
            CurveRegionParameterRange2::from_bezier_range(fragment.range().clone()),
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
    let endpoints = retained_fragment_algebraic_ray_endpoints(fragment, policy)?;
    match retained_line_fragment_segment(fragment, policy)? {
        Classification::Decided(line) => {
            return Ok(Classification::Decided(AlgebraicRayRationalFragment2 {
                curve: RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(line),
                ))?,
                retained_range: None,
                reversed: false,
                endpoints,
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
        BezierSplitFragment2::AlgebraicEndpointImages {
            start,
            end,
            reversed,
            source_curve: Some(curve),
            ..
        } => (
            RationalBezier2::try_from_subcurve(curve)?,
            Some(CurveRegionParameterRange2::new_validated(
                CurveRegionParameter2::from_bezier(start.clone()),
                CurveRegionParameter2::from_bezier(end.clone()),
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
                Some(CurveRegionParameterRange2::from_bezier_range(
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
        BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: None, ..
        }
        | BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
        | BezierSplitFragment2::Unresolved { .. } => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
    };
    let (curve, retained_range) = if let Some(range) = retained_range {
        if let Some((start, end)) = range.exact_endpoints() {
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
        endpoints,
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

fn algebraic_ray_rational_fragment_endpoint_side_signs(
    fragment: &AlgebraicRayRationalFragment2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    side_x: &Real,
    side_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<[RealSign; 2]>> {
    let mut signs = [RealSign::Zero; 2];
    for (index, endpoint) in fragment.endpoints.iter().enumerate() {
        signs[index] = match retained_point_linear_difference_to_algebraic_sign(
            endpoint, point, side_x, side_y, policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    }
    Ok(Classification::Decided(signs))
}

fn algebraic_point_on_rational_curve(
    curve: &RationalBezier2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let weight_sign = match algebraic_ray_curve_weight_sign(curve, policy) {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(_) => RealSign::Zero,
    };
    if weight_sign != RealSign::Zero {
        let controls = curve
            .control_points()
            .iter()
            .zip(curve.weights())
            .map(|(control, weight)| AlgebraicRayHomogeneousControl2 {
                x: control.x() * weight,
                y: control.y() * weight,
                weight: weight.clone(),
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
        let parameters =
            match algebraic_ray_project_selected_fiber_parameters(incidence, point, policy)? {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    parameters,
                )) => parameters,
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
            match retained_curve_region_parameter_contains(
                &parameter,
                range,
                false,
                fragment.reversed,
                policy,
            )? {
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
            policy,
        );
    }
    let weight_sign = match algebraic_ray_curve_weight_sign(&fragment.curve, policy) {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let controls = fragment
        .curve
        .control_points()
        .iter()
        .zip(fragment.curve.weights())
        .map(|(control, weight)| AlgebraicRayHomogeneousControl2 {
            x: control.x() * weight,
            y: control.y() * weight,
            weight: weight.clone(),
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
        if !side.negative || !side.positive {
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
        if !ahead.positive {
            continue;
        }
        if !ahead.negative {
            let start = side
                .first_nonzero
                .expect("a sign-changing Bernstein hull has a first sign");
            let end = side
                .last_nonzero
                .expect("a sign-changing Bernstein hull has a last sign");
            let delta = algebraic_ray_crossing_delta(start, end);
            winding = winding.checked_add(delta).ok_or_else(|| {
                CurveError::Topology("algebraic ray winding exceeds the curve counter".into())
            })?;
            continue;
        }
        let (left, right) = split_algebraic_ray_controls_at_half(&controls, &half);
        let midpoint = left
            .last()
            .expect("a rational Bezier subdivision has a midpoint control");
        let midpoint_side = match point.homogeneous_linear_difference_sign(
            &midpoint.x,
            &midpoint.y,
            &midpoint.weight,
            &side_x,
            &side_y,
            weight_sign,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if midpoint_side == RealSign::Zero {
            let midpoint_ahead = match point.homogeneous_linear_difference_sign(
                &midpoint.x,
                &midpoint.y,
                &midpoint.weight,
                direction_x,
                direction_y,
                weight_sign,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if midpoint_ahead == RealSign::Zero {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            if midpoint_ahead == RealSign::Positive {
                let before = match algebraic_ray_control_sign_hull(
                    &left,
                    point,
                    &side_x,
                    &side_y,
                    weight_sign,
                    policy,
                )? {
                    Classification::Decided(hull) => hull.last_nonzero,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let after = match algebraic_ray_control_sign_hull(
                    &right,
                    point,
                    &side_x,
                    &side_y,
                    weight_sign,
                    policy,
                )? {
                    Classification::Decided(hull) => hull.first_nonzero,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let (Some(before), Some(after)) = (before, after) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                };
                winding = winding
                    .checked_add(algebraic_ray_crossing_delta(before, after))
                    .ok_or_else(|| {
                        CurveError::Topology(
                            "algebraic ray winding exceeds the curve counter".into(),
                        )
                    })?;
            }
        }
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

fn algebraic_point_retained_rational_curve_ray_winding(
    fragment: &AlgebraicRayRationalFragment2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<i32>> {
    let range = fragment
        .retained_range
        .as_ref()
        .expect("retained algebraic ray winding requires a retained range");
    let weight_sign = match algebraic_ray_curve_weight_sign(&fragment.curve, policy) {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let side_x = -direction_y.clone();
    let side_y = direction_x.clone();
    let incidence =
        algebraic_point_rational_curve_linear_equation(&fragment.curve, point, &side_x, &side_y)?;

    if let (BezierParameter2::Algebraic(retained), Some((start, end))) =
        (point.retained_parameter(), range.as_bezier_parameters())
    {
        let range = BezierParameterRange2::new_validated(start.clone(), end.clone());
        if bivariate_fiber_strict_sign_on_parameter_range(&incidence, retained, &range, policy)?
            .is_some()
        {
            return Ok(Classification::Decided(0));
        }
    }

    let parameters =
        match algebraic_ray_project_selected_fiber_parameters(&incidence, point, policy)? {
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
        match retained_curve_region_parameter_contains(
            &parameter,
            range,
            true,
            fragment.reversed,
            policy,
        )? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => continue,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
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
        match ahead_sign {
            RealSign::Negative => continue,
            RealSign::Zero => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            RealSign::Positive => {}
        }

        let mut derivative = incidence.clone();
        let mut derivative_order = 0_usize;
        let delta = loop {
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
                if derivative_order.is_multiple_of(2) {
                    break 0_i32;
                }
                let derivative_sign =
                    multiply_algebraic_ray_signs(derivative_sign, denominator_sign);
                break match derivative_sign {
                    RealSign::Negative => -1,
                    RealSign::Positive => 1,
                    RealSign::Zero => unreachable!(),
                };
            }
            if derivative.coefficients.iter().all(|row| row.len() <= 1) {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
        };
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

fn algebraic_ray_project_selected_fiber_parameters(
    incidence: &BivariatePolynomial,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let BezierParameter2::Exact(parameter) = point.retained_parameter() else {
        let BezierParameter2::Algebraic(parameter) = point.retained_parameter() else {
            unreachable!();
        };
        return algebraic_selected_fiber_parameters(incidence, parameter, policy);
    };
    let second_count = incidence
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or_default();
    let mut coefficients = vec![Real::zero(); second_count];
    let mut first_power = Real::one();
    for row in &incidence.coefficients {
        for (coefficient, target) in row.iter().zip(&mut coefficients) {
            *target += coefficient * &first_power;
        }
        first_power *= parameter;
    }
    let mut all_zero = true;
    for coefficient in &coefficients {
        match real_sign(coefficient, policy) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => all_zero = false,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    if all_zero {
        return Ok(Classification::Decided(
            BezierAlgebraicFiberProjection2::IdenticallyZero,
        ));
    }
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(match polynomial.isolate_unit_interval_roots(policy)? {
        Classification::Decided(parameters) => {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters))
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
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

const fn algebraic_ray_crossing_delta(start: RealSign, end: RealSign) -> i32 {
    match (start, end) {
        (RealSign::Negative, RealSign::Positive) => 1,
        (RealSign::Positive, RealSign::Negative) => -1,
        (RealSign::Negative, RealSign::Negative) | (RealSign::Positive, RealSign::Positive) => 0,
        (RealSign::Zero, _) | (_, RealSign::Zero) => 0,
    }
}

fn algebraic_ray_curve_weight_sign(
    curve: &RationalBezier2,
    policy: &CurveContext,
) -> Classification<RealSign> {
    let Some(first) = real_sign(&curve.weights()[0], policy) else {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    };
    if first == RealSign::Zero {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    }
    for weight in &curve.weights()[1..] {
        match real_sign(weight, policy) {
            Some(sign) if sign == first => {}
            Some(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    Classification::Decided(first)
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
    evaluators: &[Vec<Option<RationalBezier2>>],
    point: &Point2,
    policy: &CurveContext,
    roles: Option<&[CurveRegionLoopRole]>,
    fill_rules: Option<&[FillRule]>,
) -> CurveResult<Classification<RegionPointLocation>> {
    if boundary_loops.len() != evaluators.len() {
        return Err(CurveError::Topology(
            "retained region evaluator cache loop count is inconsistent".into(),
        ));
    }
    if roles.is_some_and(|roles| roles.len() != boundary_loops.len())
        || fill_rules.is_some_and(|rules| rules.len() != boundary_loops.len())
    {
        return Err(CurveError::Topology(
            "retained region loop semantics are inconsistent with boundary loops".into(),
        ));
    }
    let mut inside = false;
    let mut signed_depth = 0_i32;
    for (index, (boundary_loop, evaluators)) in boundary_loops.iter().zip(evaluators).enumerate() {
        let fill_rule = fill_rules.map_or(FillRule::EvenOdd, |rules| rules[index]);
        match classify_point_against_retained_loop_with_fill_rule(
            boundary_loop,
            evaluators,
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
    evaluators: &[Option<RationalBezier2>],
    point: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    classify_point_against_retained_loop_with_fill_rule(
        boundary_loop,
        evaluators,
        point,
        FillRule::EvenOdd,
        policy,
    )
}

fn classify_point_against_retained_loop_with_fill_rule(
    boundary_loop: &CurveRegionBoundaryLoop2,
    evaluators: &[Option<RationalBezier2>],
    point: &Point2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    if boundary_loop.fragments().len() != evaluators.len() {
        return Err(CurveError::Topology(
            "retained region evaluator cache fragment count is inconsistent".into(),
        ));
    }
    if let Classification::Decided(bounds) = retained_loop_query_bounds(boundary_loop, policy)
        && bounds.contains_point(point, policy) == Classification::Decided(false)
    {
        return Ok(Classification::Decided(ContourPointLocation::Outside));
    }
    for (fragment, evaluator) in boundary_loop.fragments().iter().zip(evaluators) {
        if let BezierSplitFragment2::Materialized { curve, .. } = fragment
            && matches!(
                subcurve_control_hull_contains_point(curve, point, policy),
                Classification::Decided(false)
            )
        {
            continue;
        }
        match retained_fragment_contains_point(fragment, evaluator.as_ref(), point, policy)? {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            Classification::Decided(false) | Classification::Uncertain(_) => {}
        }
    }
    let mut last_reason = UncertaintyReason::Boundary;
    for ray in ray_candidates(point) {
        match classify_point_with_retained_ray(boundary_loop, point, &ray, fill_rule, policy)? {
            Classification::Decided(location) => {
                return Ok(Classification::Decided(location));
            }
            Classification::Uncertain(reason) => last_reason = reason,
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
        BezierSplitFragment2::AlgebraicEndpointImages {
            start,
            end,
            source_curve: Some(_),
            ..
        } => {
            let Some(evaluator) = evaluator else {
                return Err(CurveError::Topology(
                    "retained algebraic source evaluator cache is incomplete".into(),
                ));
            };
            match evaluator.point_incidence(point, policy) {
                Ok(RationalBezierPointIncidence2::EntireCurve) => Ok(Classification::Decided(true)),
                Ok(RationalBezierPointIncidence2::Parameters(parameters)) => {
                    for parameter in parameters {
                        match retained_parameter_contains(
                            &parameter, start, end, false, false, policy,
                        )? {
                            Classification::Decided(true) => {
                                return Ok(Classification::Decided(true));
                            }
                            Classification::Decided(false) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    Ok(Classification::Decided(false))
                }
                Err(ExactCurveError::Blocked(blocker)) => {
                    Ok(Classification::Uncertain(blocker.reason()))
                }
                Err(ExactCurveError::Invalid { cause, .. }) => Err(cause),
            }
        }
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            match fragment.parallel().point_incidence(point, policy)? {
                Classification::Decided(crate::BezierParallelIncidence2::EntireCurve) => {
                    Ok(Classification::Decided(true))
                }
                Classification::Decided(crate::BezierParallelIncidence2::Parameters(
                    parameters,
                )) => {
                    for parameter in parameters {
                        match retained_parameter_contains(
                            &parameter,
                            fragment.range().start(),
                            fragment.range().end(),
                            false,
                            false,
                            policy,
                        )? {
                            Classification::Decided(true) => {
                                return Ok(Classification::Decided(true));
                            }
                            Classification::Decided(false) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    Ok(Classification::Decided(false))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        }
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let parameters = if let Some(curve) = fragment.rational_curve() {
                match curve.point_incidence(point, policy) {
                    Ok(RationalBezierPointIncidence2::EntireCurve) => {
                        return Ok(Classification::Decided(true));
                    }
                    Ok(RationalBezierPointIncidence2::Parameters(parameters)) => parameters,
                    Err(ExactCurveError::Blocked(blocker)) => {
                        return Ok(Classification::Uncertain(blocker.reason()));
                    }
                    Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
                }
            } else {
                let parallel = fragment
                    .analytic_parallel()
                    .expect("a selected-fiber source is rational or analytic");
                match parallel.point_incidence(point, policy)? {
                    Classification::Decided(crate::BezierParallelIncidence2::EntireCurve) => {
                        return Ok(Classification::Decided(true));
                    }
                    Classification::Decided(crate::BezierParallelIncidence2::Parameters(
                        parameters,
                    )) => parameters,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            for parameter in parameters {
                match retained_curve_region_parameter_contains(
                    &parameter,
                    fragment.range(),
                    false,
                    fragment.is_reversed(),
                    policy,
                )? {
                    Classification::Decided(true) => {
                        return Ok(Classification::Decided(true));
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Ok(Classification::Decided(false))
        }
        BezierSplitFragment2::AlgebraicChord(chord) => chord.contains_point(point, policy),
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            fragment.contains_point(point, policy)
        }
        BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: None, ..
        }
        | BezierSplitFragment2::Unresolved { .. } => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
    }
}

#[derive(Clone, Copy)]
struct RetainedRayOriginContact<'a> {
    fragment_index: Option<usize>,
    parameter: Option<&'a CurveRegionParameter2>,
    crossing_direction: BezierLineCrossingDirection,
    tangent_contacts: Option<&'a [crate::rational_bezier::RationalQuadraticCircleTangentContact2]>,
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
        | BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: Some(BezierSubcurve2::Rational(curve)),
            ..
        } => curve.retained_circular_conic(),
        BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: Some(BezierSubcurve2::RationalQuadratic(curve)),
            ..
        } => curve.retained_circular_conic(),
        BezierSplitFragment2::SelectedFiber(fragment) => fragment
            .rational_curve()
            .and_then(RationalBezier2::retained_circular_conic),
        BezierSplitFragment2::Materialized { .. }
        | BezierSplitFragment2::AlgebraicEndpointImages { .. }
        | BezierSplitFragment2::AnalyticParallel(_)
        | BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
        | BezierSplitFragment2::Unresolved { .. } => None,
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
    classify_point_with_retained_ray_skipping_origin(
        boundary_loop,
        point,
        ray,
        fill_rule,
        None,
        policy,
    )
}

fn classify_point_with_retained_ray_skipping_origin(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &Point2,
    ray: &BezierRay2,
    fill_rule: FillRule,
    skipped_origin: Option<RetainedRayOriginContact<'_>>,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    let direction_x = &ray.direction_x;
    let direction_y = &ray.direction_y;
    let mut winding = 0_i32;
    let mut source_origin_contact_was_skipped = false;
    for (fragment_index, fragment) in boundary_loop.fragments().iter().enumerate() {
        let exact_parallel_curve = match fragment {
            BezierSplitFragment2::AnalyticParallel(fragment) => match fragment
                .parallel()
                .exact_rational_parallel_component(policy)
            {
                Ok(Classification::Decided(Some(curve))) => Some(BezierSubcurve2::Rational(curve)),
                Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => None,
            },
            BezierSplitFragment2::Materialized { .. }
            | BezierSplitFragment2::AlgebraicEndpointImages { .. }
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::Unresolved { .. } => None,
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
                None => chord.forward_ray_winding_delta(point, direction_x, direction_y, policy)?,
            };
            match result {
                Classification::Decided(delta) => winding += delta,
                Classification::Uncertain(UncertaintyReason::Boundary) => {
                    return Ok(Classification::Decided(ContourPointLocation::Boundary));
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
                None => {
                    fragment.forward_ray_winding_delta(point, direction_x, direction_y, policy)?
                }
            };
            match result {
                Classification::Decided(delta) => winding += delta,
                Classification::Uncertain(UncertaintyReason::Boundary) => {
                    return Ok(Classification::Decided(ContourPointLocation::Boundary));
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
            | BezierSplitFragment2::AlgebraicEndpointImages { .. }
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::Unresolved { .. } => None,
        };
        if let Some((parallel, reversed, ordinary_range, selected_range)) = procedural_parallel {
            let certified_origin_parameter =
                skipped_origin.and_then(|origin| {
                    if origin.fragment_index == Some(fragment_index) {
                        return origin
                            .parameter
                            .and_then(CurveRegionParameter2::as_bezier_parameter)
                            .and_then(BezierParameter2::as_exact);
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
                certified_origin_parameter.map(|parameter| (parameter, origin.crossing_direction))
            });
            let relation = match parallel.relation_to_supporting_line_with_certified_crossing(
                &ray.line,
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
                        let retained = if let Some(range) = selected_range {
                            retained_curve_region_parameter_contains(
                                contact.parameter(),
                                range,
                                true,
                                reversed,
                                policy,
                            )?
                        } else {
                            let range = ordinary_range
                                .expect("an ordinary analytic fragment retains its range");
                            retained_parameter_contains(
                                contact.parameter(),
                                range.start(),
                                range.end(),
                                true,
                                reversed,
                                policy,
                            )?
                        };
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
                        match parallel.supporting_line_parameter_order(
                            contact.parameter(),
                            &ray.line,
                            policy,
                        )? {
                            Classification::Decided(std::cmp::Ordering::Greater) => {
                                if contact.kind() != BezierLineContactKind::Crossing {
                                    continue;
                                }
                                let Some(delta) = line_contact_winding_delta(&contact, reversed)
                                else {
                                    return Ok(Classification::Uncertain(
                                        UncertaintyReason::Unsupported,
                                    ));
                                };
                                winding += delta;
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
            BezierSplitFragment2::AlgebraicEndpointImages {
                reversed,
                start,
                end,
                source_curve: Some(curve),
                ..
            } => (
                curve,
                Some(CurveRegionParameterRange2::new_validated(
                    CurveRegionParameter2::from_bezier(start.clone()),
                    CurveRegionParameter2::from_bezier(end.clone()),
                )),
                *reversed,
            ),
            BezierSplitFragment2::AlgebraicEndpointImages {
                source_curve: None, ..
            }
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::Unresolved { .. } => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => (
                exact_parallel_curve
                    .as_ref()
                    .expect("exact analytic component was selected above"),
                Some(CurveRegionParameterRange2::from_bezier_range(
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
        if !subcurve_control_hull_may_be_ahead(curve, point, direction_x, direction_y, policy) {
            continue;
        }
        let control_hull_order =
            subcurve_control_hull_strict_order(curve, point, direction_x, direction_y, policy);
        let certified_source_crossing = skipped_origin.and_then(|origin| {
            (origin.fragment_index == Some(fragment_index))
                .then(|| {
                    origin
                        .parameter
                        .and_then(CurveRegionParameter2::as_bezier_parameter)
                        .and_then(BezierParameter2::as_exact)
                        .map(|parameter| (parameter, origin.crossing_direction))
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
                retained_circle.then(|| {
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
            Ok(None) => subcurve_relation_to_line_with_contacts(
                curve,
                &ray.line,
                Some((direction_x, direction_y)),
                policy,
            ),
            Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
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
                        let is_origin = contact.supporting_line_parameter().map_or_else(
                            || {
                                origin
                                    .parameter
                                    .and_then(CurveRegionParameter2::as_bezier_parameter)
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
                            },
                            |line_parameter| {
                                compare_reals(line_parameter, &Real::zero(), policy).map_or_else(
                                    || {
                                        origin
                                            .parameter
                                            .and_then(CurveRegionParameter2::as_bezier_parameter)
                                            .map_or(
                                                Ok(Classification::Uncertain(
                                                    UncertaintyReason::Boundary,
                                                )),
                                                |parameter| {
                                                    retained_parameters_equal(
                                                        contact.parameter(),
                                                        parameter,
                                                        policy,
                                                    )
                                                },
                                            )
                                    },
                                    |order| {
                                        Ok(Classification::Decided(
                                            order == std::cmp::Ordering::Equal,
                                        ))
                                    },
                                )
                            },
                        )?;
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
                    let retained = if let Some(range) = range.as_ref() {
                        retained_curve_region_parameter_contains(
                            contact.parameter(),
                            range,
                            true,
                            reversed,
                            policy,
                        )?
                    } else {
                        retained_parameter_contains(
                            contact.parameter(),
                            &BezierParameter2::Exact(Real::zero()),
                            &BezierParameter2::Exact(Real::one()),
                            true,
                            false,
                            policy,
                        )?
                    };
                    match retained {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => continue,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
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
                    match ahead {
                        Classification::Decided(std::cmp::Ordering::Greater) => {
                            if contact.kind() != BezierLineContactKind::Crossing {
                                continue;
                            }
                            let Some(delta) = line_contact_winding_delta(&contact, reversed) else {
                                return Ok(Classification::Uncertain(
                                    UncertaintyReason::Unsupported,
                                ));
                            };
                            winding += delta;
                        }
                        Classification::Decided(std::cmp::Ordering::Equal) => {
                            if skipped_origin.is_some()
                                && contact.kind() == BezierLineContactKind::Crossing
                            {
                                continue;
                            }
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
    if skipped_origin.is_some_and(|origin| origin.fragment_index.is_some())
        && !source_origin_contact_was_skipped
    {
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    }
    Ok(Classification::Decided(winding_location(
        winding, fill_rule,
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

fn retained_parameter_contains(
    parameter: &BezierParameter2,
    start: &BezierParameter2,
    end: &BezierParameter2,
    half_open: bool,
    reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let start_order = match parameter.cmp_by_refinement(start, policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_order = match parameter.cmp_by_refinement(end, policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let after_start = start_order == std::cmp::Ordering::Greater
        || (start_order == std::cmp::Ordering::Equal && (!half_open || !reversed));
    let before_end = end_order == std::cmp::Ordering::Less
        || (end_order == std::cmp::Ordering::Equal && (!half_open || reversed));
    Ok(Classification::Decided(after_start && before_end))
}

fn retained_curve_region_parameter_contains(
    parameter: &BezierParameter2,
    range: &CurveRegionParameterRange2,
    half_open: bool,
    reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let parameter = CurveRegionParameter2::from_bezier(parameter.clone());
    let start_order = match parameter.cmp_by_refinement(range.start(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_order = match parameter.cmp_by_refinement(range.end(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let after_start = start_order == std::cmp::Ordering::Greater
        || (start_order == std::cmp::Ordering::Equal && (!half_open || !reversed));
    let before_end = end_order == std::cmp::Ordering::Less
        || (end_order == std::cmp::Ordering::Equal && (!half_open || reversed));
    Ok(Classification::Decided(after_start && before_end))
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
        bounds = match bounds.union(&fragment_bounds, policy) {
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
        bounds = match bounds.union(&fragment_bounds, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
    }
    Classification::Decided(bounds)
}

fn retained_fragment_query_bounds(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => subcurve_query_bounds(curve, policy),
        BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: Some(curve),
            ..
        } => subcurve_query_bounds(curve, policy),
        BezierSplitFragment2::AnalyticParallel(fragment) => fragment
            .parallel()
            .conservative_bounds(policy)
            .unwrap_or_else(|_| Classification::Uncertain(UncertaintyReason::Unsupported)),
        BezierSplitFragment2::AlgebraicChord(chord) => chord
            .conservative_bounds(policy)
            .unwrap_or_else(|_| Classification::Uncertain(UncertaintyReason::Unsupported)),
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => fragment
            .conservative_bounds()
            .unwrap_or_else(|_| Classification::Uncertain(UncertaintyReason::Unsupported)),
        BezierSplitFragment2::SelectedFiber(fragment) => fragment
            .conservative_bounds(policy)
            .unwrap_or_else(|_| Classification::Uncertain(UncertaintyReason::Unsupported)),
        BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: None, ..
        }
        | BezierSplitFragment2::Unresolved { .. } => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
    }
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
                    let one = BezierParameter2::Exact(Real::one());
                    match contact.parameter().cmp_by_interval(&one, policy)? {
                        Classification::Decided(std::cmp::Ordering::Equal) => continue,
                        Classification::Decided(_) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
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
                            if contact.kind() != BezierLineContactKind::Crossing {
                                continue;
                            }
                            let Some(delta) = line_contact_winding_delta(&contact, false) else {
                                return Ok(Classification::Uncertain(
                                    UncertaintyReason::Unsupported,
                                ));
                            };
                            winding += delta;
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
        real_sign(&projection, policy) != Some(RealSign::Negative)
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
        let current = match real_sign(&projection, policy)? {
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

fn line_contact_winding_delta(contact: &BezierLineContact, reversed: bool) -> Option<i32> {
    let delta = match contact.crossing_direction()? {
        BezierLineCrossingDirection::NegativeToPositive => 1,
        BezierLineCrossingDirection::PositiveToNegative => -1,
    };
    Some(if reversed { -delta } else { delta })
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
    let ordering = match curve {
        BezierSubcurve2::Quadratic(curve) => Ok(polynomial_image_coordinate_order(
            &curve.point_at_algebraic_parameter(parameter, policy)?,
            use_x,
            origin_coordinate,
            policy,
        )),
        BezierSubcurve2::Cubic(curve) => Ok(polynomial_image_coordinate_order(
            &curve.point_at_algebraic_parameter(parameter, policy)?,
            use_x,
            origin_coordinate,
            policy,
        )),
        BezierSubcurve2::RationalQuadratic(curve) => rational_image_coordinate_order(
            &curve.point_at_algebraic_parameter(parameter, policy)?,
            use_x,
            origin_coordinate,
            policy,
        ),
        BezierSubcurve2::Rational(curve) => rational_image_coordinate_order(
            &curve.point_at_algebraic_parameter(parameter, policy)?,
            use_x,
            origin_coordinate,
            policy,
        ),
    }?;
    Ok(ordering.map(|ordering| {
        if direction_sign == RealSign::Negative {
            ordering.reverse()
        } else {
            ordering
        }
    }))
}

fn polynomial_image_coordinate_order(
    image: &crate::BezierAlgebraicPointImage2,
    use_x: bool,
    origin: &Real,
    policy: &CurveContext,
) -> Classification<std::cmp::Ordering> {
    let coordinate = if use_x { image.x() } else { image.y() };
    coordinate.map_or(
        Classification::Uncertain(UncertaintyReason::Unsupported),
        |coordinate| coordinate.compare_to_real(origin, policy),
    )
}

fn rational_image_coordinate_order(
    image: &crate::RationalBezierAlgebraicPointImage2,
    use_x: bool,
    origin: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    image.coordinate_order_to_real(use_x, origin, policy)
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
        BezierSubcurve2::Quadratic(curve) => Aabb2::from_points(curve.control_points(), policy),
        BezierSubcurve2::Cubic(curve) => Aabb2::from_points(curve.control_points(), policy),
        BezierSubcurve2::RationalQuadratic(curve)
            if curve.common_nonzero_weight_sign(policy).is_some() =>
        {
            Aabb2::from_points(curve.control_points(), policy)
        }
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(policy),
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
        BezierSubcurve2::Rational(curve) => curve.point_at_classified(&parameter, policy),
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
        BezierSubcurve2::Rational(curve) => curve.relation_to_line_with_contacts(line, policy),
    }
}

impl BezierSubcurve2 {
    /// Returns exact signed-area contribution when implemented for this curve family.
    pub fn signed_area_contribution(
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

    /// Returns exact signed-area and first-moment contributions when the
    /// fragment has an implemented symbolic integral.
    pub fn area_moments_contribution(
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

fn certified_measurement<T>(value: T) -> CurveOutcome<Classification<Option<T>>> {
    CurveOutcome::new(
        Classification::Decided(Some(value)),
        CurveCertainty::Certified,
    )
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
mod tests {
    use super::*;
    use std::cmp::Ordering;

    use crate::{
        BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial,
        CurveCertainty,
    };
    use crate::{
        CircularArc2, CubicBezier2, Curve2, CurvePath2, QuadraticBezier2, RationalQuadraticBezier2,
    };

    fn p(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    fn parallel_pair_fillet_region(
        previous_retained: bool,
        next_retained: bool,
        reversed: bool,
        policy: &CurveContext,
    ) -> CurveRegion2 {
        let fragment = |start: Point2, end: Point2, retained: bool| {
            let half = (Real::one() / Real::from(2_i8)).unwrap();
            let curve = QuadraticBezier2::new(start.clone(), start.lerp(&end, half), end.clone());
            if retained {
                let parallel = curve
                    .parallel_left(Real::zero())
                    .expect("the exact-line analytic parallel is valid");
                let range = BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                );
                let Classification::Decided(fragment) =
                    crate::BezierParallelFragment2::try_new(parallel, range, policy)
                        .expect("the complete analytic range is valid")
                else {
                    panic!("the exact analytic range must be decided");
                };
                BezierSplitFragment2::AnalyticParallel(fragment)
            } else {
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(curve),
                }
            }
        };
        let mut fragments = vec![
            fragment(p(0, 0), p(4, 0), previous_retained),
            fragment(p(4, 0), p(4, 4), next_retained),
            fragment(p(4, 4), p(0, 0), false),
        ];
        let interior_side = if reversed {
            fragments = fragments
                .iter()
                .rev()
                .map(|fragment| fragment.reversed().expect("the exact fixture reverses"))
                .collect();
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        };
        let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
            .expect("the parallel-pair fixture closes exactly");
        CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![interior_side],
        )
        .expect("the parallel-pair fixture has authored topology")
    }

    #[test]
    fn direct_mixed_and_retained_parallel_pairs_share_the_fillet_kernel() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (previous_retained, next_retained) in [(false, true), (true, false), (true, true)] {
                for reversed in [false, true] {
                    let region = parallel_pair_fillet_region(
                        previous_retained,
                        next_retained,
                        reversed,
                        &policy,
                    );
                    let corner = if reversed { 2 } else { 1 };
                    let result = region
                        .fillet_loop_vertex_by_radius(
                            0,
                            corner,
                            Real::one(),
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap_or_else(|error| {
                            panic!(
                                "the unified parallel pair must fillet: policy={policy:?}, previous_retained={previous_retained}, next_retained={next_retained}, reversed={reversed}, error={error:?}"
                            )
                        });
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    let CurveCornerSolutions2::Unique(filleted) = result.value else {
                        panic!(
                            "the unified parallel pair must have one fillet: policy={policy:?}, previous_retained={previous_retained}, next_retained={next_retained}, reversed={reversed}"
                        );
                    };
                    assert_eq!(
                        filleted
                            .classify_point(&p(3, 1), &policy)
                            .expect("the unified parallel-pair fillet remains classifiable")
                            .into_value(),
                        Classification::Decided(RegionPointLocation::Inside),
                    );
                }
            }
        }
    }

    fn sqrt_half_algebraic_parameter(policy: &CurveContext) -> BezierParameter2 {
        let polynomial = BezierParameterPolynomial::try_new_power_basis(
            vec![Real::from(-1_i8), Real::zero(), Real::from(2_i8)],
            policy,
        )
        .expect("the quadratic parameter polynomial is valid");
        let Classification::Decided(polynomial) = polynomial else {
            panic!("the exact polynomial must be decided");
        };
        let interval = BezierParameterInterval::try_new(
            (Real::from(2_i8) / Real::from(3_i8)).unwrap(),
            (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
            policy,
        )
        .expect("the isolating interval is valid");
        let Classification::Decided(interval) = interval else {
            panic!("the exact interval must be decided");
        };
        let parameter = BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)
            .expect("sqrt(1/2) has one root in the supplied interval");
        let Classification::Decided(parameter) = parameter else {
            panic!("the exact algebraic parameter must be decided");
        };
        BezierParameter2::algebraic(parameter)
    }

    #[derive(Clone, Copy)]
    enum SelectedCircleFilletNeighbor2 {
        RationalArc(i8),
        SelectedCircle,
        AnalyticParallel(bool),
        DirectBezier,
    }

    fn selected_circle_neighbor_region(
        policy: &CurveContext,
        neighbor: SelectedCircleFilletNeighbor2,
        reversed: bool,
    ) -> CurveRegion2 {
        let center_parameter = sqrt_half_algebraic_parameter(policy);
        let BezierParameter2::Algebraic(center_parameter) = &center_parameter else {
            panic!("sqrt(1/2) must remain an isolated algebraic parameter");
        };
        let center_source = RationalBezier2::try_new(
            vec![p(0, 0), p(0, 0), p(1, 0)],
            vec![Real::one(), Real::one(), Real::one()],
        )
        .expect("the selected center source is a valid rational quadratic");
        let center = RationalBezierIntersectionPointEvidence2::Algebraic(
            center_source
                .point_at_algebraic_parameter(center_parameter, policy)
                .expect("the selected center has an exact rational image"),
        );
        let Classification::Decided(Some(support)) =
            crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                &center,
                (1, 0),
                Real::one(),
                true,
                policy,
            )
            .expect("the selected center defines an exact clockwise semicircle")
        else {
            panic!("the nonzero selected semicircle must be decided");
        };

        let alpha = (Real::one() / Real::from(2_i8)).unwrap();
        let half_sqrt_two = alpha.clone().sqrt().unwrap();
        let start = Point2::new(&alpha + Real::one(), Real::zero());
        let join = Point2::new(&alpha - Real::one(), Real::zero());
        let arc_end = Point2::new(alpha.clone(), Real::one());
        let neighbor = match neighbor {
            SelectedCircleFilletNeighbor2::RationalArc(homogeneous_scale) => {
                let scale = Real::from(homogeneous_scale);
                let arc = RationalQuadraticBezier2::try_new(
                    join.clone(),
                    Point2::new(alpha.clone(), Real::zero()),
                    arc_end.clone(),
                    scale.clone(),
                    &scale * half_sqrt_two,
                    scale,
                )
                .expect("the retained quarter circle has a valid homogeneous gauge");
                assert!(
                    matches!(
                        crate::arc_bezier::rational_quadratic_circular_arc(&arc, policy),
                        Ok(Classification::Decided(Some(_)))
                    ),
                    "the authored exact quarter circle must promote as circular"
                );
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::RationalQuadratic(arc),
                }
            }
            SelectedCircleFilletNeighbor2::SelectedCircle => {
                let neighbor_center_source = RationalBezier2::try_new(
                    vec![p(0, 1), p(0, 1), p(-1, 1)],
                    vec![Real::one(), Real::one(), Real::one()],
                )
                .expect("the neighboring selected center source is a valid rational quadratic");
                let neighbor_center = RationalBezierIntersectionPointEvidence2::Algebraic(
                    neighbor_center_source
                        .point_at_algebraic_parameter(center_parameter, policy)
                        .expect("the neighboring center has an exact rational image"),
                );
                let Classification::Decided(Some(neighbor_support)) =
                    crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                        &neighbor_center,
                        (0, -1),
                        Real::one(),
                        false,
                        policy,
                    )
                    .expect("the neighboring selected center defines an exact semicircle")
                else {
                    panic!("the neighboring selected semicircle must be decided");
                };
                let half = (Real::one() / Real::from(2_i8)).unwrap();
                let Classification::Decided(neighbor_fragment) =
                    crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                        neighbor_support,
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                            Real::zero(),
                        ),
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(half),
                        false,
                        policy,
                    )
                    .expect("the neighboring selected quarter has a valid range")
                else {
                    panic!("the neighboring selected quarter must be decided");
                };
                BezierSplitFragment2::AlgebraicCuspSemicircle(neighbor_fragment)
            }
            SelectedCircleFilletNeighbor2::AnalyticParallel(curved) => {
                let analytic = if curved {
                    QuadraticBezier2::new(
                        join.clone(),
                        Point2::new(alpha.clone(), Real::zero()),
                        arc_end.clone(),
                    )
                    .parallel_left(Real::zero())
                    .expect("the neighboring curved analytic parallel is valid")
                } else {
                    let half = (Real::one() / Real::from(2_i8)).unwrap();
                    QuadraticBezier2::new(join.clone(), join.lerp(&arc_end, half), arc_end.clone())
                        .parallel_left(Real::zero())
                        .expect("the neighboring exact-line parallel is valid")
                };
                let range = BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                );
                let Classification::Decided(analytic) =
                    crate::BezierParallelFragment2::try_new(analytic, range, policy)
                        .expect("the neighboring analytic range is valid")
                else {
                    panic!("the neighboring analytic fragment must be decided");
                };
                BezierSplitFragment2::AnalyticParallel(analytic)
            }
            SelectedCircleFilletNeighbor2::DirectBezier => {
                let one_quarter = (Real::one() / Real::from(4_i8)).unwrap();
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                        join.clone(),
                        join.lerp(&arc_end, one_quarter),
                        arc_end.clone(),
                    )),
                }
            }
        };
        let mut fragments = vec![
            BezierSplitFragment2::AlgebraicCuspSemicircle(
                crate::BezierAlgebraicCuspSemicircleFragment2::full(support, policy),
            ),
            neighbor,
            quadratic_fragment(
                arc_end,
                Point2::new(
                    &alpha + (Real::one() / Real::from(2_i8)).unwrap(),
                    (Real::one() / Real::from(2_i8)).unwrap(),
                ),
                start,
            ),
        ];
        let interior_side = if reversed {
            fragments = fragments
                .iter()
                .rev()
                .map(|fragment| fragment.reversed().expect("the exact fixture reverses"))
                .collect();
            CurveBoundaryInteriorSide2::Left
        } else {
            CurveBoundaryInteriorSide2::Right
        };
        let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
            .expect("mixed selected-circle/rational-arc endpoints close exactly");
        CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![interior_side],
        )
        .expect("the mixed exact loop has authored topology")
    }

    fn selected_fillet_disjoint_square(policy: &CurveContext) -> CurveRegion2 {
        CurveRegion2::new(vec![
            CurveRegionBoundaryLoop2::new(
                vec![
                    quadratic_fragment(p(4, 4), p(5, 4), p(6, 4)),
                    quadratic_fragment(p(6, 4), p(6, 5), p(6, 6)),
                    quadratic_fragment(p(6, 6), p(5, 6), p(4, 6)),
                    quadratic_fragment(p(4, 6), p(4, 5), p(4, 4)),
                ],
                policy,
            )
            .expect("the disjoint exact loop closes"),
        ])
        .expect("one disjoint exact loop")
    }

    #[test]
    fn selected_circle_and_retained_rational_arc_fillet_exactly() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for homogeneous_scale in [1_i8, 2_i8] {
                for reversed in [false, true] {
                    let region = selected_circle_neighbor_region(
                        &policy,
                        SelectedCircleFilletNeighbor2::RationalArc(homogeneous_scale),
                        reversed,
                    );
                    let fragments = region.boundary_loops()[0].fragments();
                    let corner = (0..fragments.len())
                        .find(|index| {
                            let previous =
                                &fragments[(index + fragments.len() - 1) % fragments.len()];
                            let next = &fragments[*index];
                            matches!(
                                (previous, next),
                                (
                                    BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                                    BezierSplitFragment2::Materialized {
                                        curve: BezierSubcurve2::RationalQuadratic(_),
                                        ..
                                    }
                                ) | (
                                    BezierSplitFragment2::Materialized {
                                        curve: BezierSubcurve2::RationalQuadratic(_),
                                        ..
                                    },
                                    BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                                )
                            )
                        })
                        .expect("the fixture retains its mixed circular corner");
                    let result = region
                        .fillet_loop_vertex_by_radius(
                            0,
                            corner,
                            (Real::one() / Real::from(10_i8)).unwrap(),
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap_or_else(|error| {
                            panic!(
                                "the mixed circular corner must fillet exactly: policy={policy:?}, scale={homogeneous_scale}, reversed={reversed}, error={error:?}"
                            )
                        });
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    let CurveCornerSolutions2::Unique(filleted) = result.value else {
                        panic!(
                            "the mixed circular corner must have one retained fillet: policy={policy:?}, scale={homogeneous_scale}, reversed={reversed}"
                        );
                    };
                    assert_eq!(
                        filleted.boundary_loops()[0]
                            .fragments()
                            .iter()
                            .filter(|fragment| matches!(
                                fragment,
                                BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                            ))
                            .count(),
                        2,
                    );
                    assert_eq!(
                        filleted
                            .classify_point(&p(0, 0), &policy)
                            .expect("the retained fillet remains classifiable")
                            .into_value(),
                        Classification::Decided(RegionPointLocation::Inside),
                    );
                    if homogeneous_scale == 1 && !reversed {
                        let disjoint = selected_fillet_disjoint_square(&policy);
                        let replay = filleted
                            .boolean_regions(&disjoint, &policy)
                            .expect("the mixed retained fillet re-enters the Boolean kernel");
                        assert_eq!(replay.certainty, CurveCertainty::Certified);
                        assert_eq!(replay.value.union().boundary_loops().len(), 2);
                        assert!(replay.value.intersection().is_empty());
                    }
                }
            }
        }
    }

    #[test]
    fn selected_circle_pair_with_rationalizable_support_fillet_exactly() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let region = selected_circle_neighbor_region(
                    &policy,
                    SelectedCircleFilletNeighbor2::SelectedCircle,
                    reversed,
                );
                let fragments = region.boundary_loops()[0].fragments();
                let corner = (0..fragments.len())
                    .find(|index| {
                        let previous = &fragments[(index + fragments.len() - 1) % fragments.len()];
                        let next = &fragments[*index];
                        matches!(
                            (previous, next),
                            (
                                BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                                BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                            )
                        )
                    })
                    .expect("the fixture retains its selected-circle pair corner");
                let result = region
                    .fillet_loop_vertex_by_radius(
                        0,
                        corner,
                        (Real::one() / Real::from(10_i8)).unwrap(),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the selected-circle pair must fillet exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert_eq!(result.certainty, CurveCertainty::Certified);
                let CurveCornerSolutions2::Unique(filleted) = result.value else {
                    panic!(
                        "the selected-circle pair must have one retained fillet: policy={policy:?}, reversed={reversed}"
                    );
                };
                assert_eq!(
                    filleted.boundary_loops()[0]
                        .fragments()
                        .iter()
                        .filter(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        ))
                        .count(),
                    3,
                );
                assert_eq!(
                    filleted
                        .classify_point(&p(0, 0), &policy)
                        .expect("the selected-circle pair fillet remains classifiable")
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Inside),
                );
                if !reversed {
                    let replay = filleted
                        .boolean_regions(&selected_fillet_disjoint_square(&policy), &policy)
                        .expect("the selected-circle pair fillet re-enters the Boolean kernel");
                    assert_eq!(replay.certainty, CurveCertainty::Certified);
                    assert_eq!(replay.value.union().boundary_loops().len(), 2);
                    assert!(replay.value.intersection().is_empty());
                }
            }
        }
    }

    #[test]
    fn selected_circle_and_analytic_parallel_fillet_exactly() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (curved, reversed) in [false, true]
                .into_iter()
                .flat_map(|curved| [false, true].map(|reversed| (curved, reversed)))
            {
                let region = selected_circle_neighbor_region(
                    &policy,
                    SelectedCircleFilletNeighbor2::AnalyticParallel(curved),
                    reversed,
                );
                let fragments = region.boundary_loops()[0].fragments();
                let corner = (0..fragments.len())
                    .find(|index| {
                        let previous = &fragments[(index + fragments.len() - 1) % fragments.len()];
                        let next = &fragments[*index];
                        matches!(
                            (previous, next),
                            (
                                BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                                BezierSplitFragment2::AnalyticParallel(_)
                            ) | (
                                BezierSplitFragment2::AnalyticParallel(_),
                                BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                            )
                        )
                    })
                    .expect("the fixture retains its selected-circle/analytic corner");
                let result = region
                    .fillet_loop_vertex_by_radius(
                        0,
                        corner,
                        (Real::one() / Real::from(10_i8)).unwrap(),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the selected-circle/analytic corner must fillet exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert_eq!(result.certainty, CurveCertainty::Certified);
                let CurveCornerSolutions2::Unique(filleted) = result.value else {
                    panic!(
                        "the selected-circle/analytic corner must have one retained fillet: policy={policy:?}, reversed={reversed}"
                    );
                };
                assert_eq!(
                    filleted.boundary_loops()[0]
                        .fragments()
                        .iter()
                        .filter(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        ))
                        .count(),
                    2,
                );
                assert_eq!(
                    filleted
                        .classify_point(&p(0, 0), &policy)
                        .expect("the selected-circle/analytic fillet remains classifiable")
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Inside),
                );
                assert_eq!(
                    filleted
                        .classify_point(&p(-1, 0), &policy)
                        .expect("the selected-circle/analytic exterior remains classifiable")
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Outside),
                );
                if !curved {
                    let replay = filleted
                        .boolean_regions(&selected_fillet_disjoint_square(&policy), &policy)
                        .expect("the selected-circle/analytic fillet re-enters the Boolean kernel");
                    assert_eq!(replay.certainty, CurveCertainty::Certified);
                    assert_eq!(replay.value.union().boundary_loops().len(), 2);
                    assert!(replay.value.intersection().is_empty());
                }
            }
        }
    }

    #[test]
    fn selected_circle_and_direct_bezier_share_the_parallel_fillet_kernel() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let region = selected_circle_neighbor_region(
                    &policy,
                    SelectedCircleFilletNeighbor2::DirectBezier,
                    reversed,
                );
                let fragments = region.boundary_loops()[0].fragments();
                let corner = if reversed { 2 } else { 1 };
                let previous = &fragments[(corner + fragments.len() - 1) % fragments.len()];
                let next = &fragments[corner];
                assert!(matches!(
                    (previous, next),
                    (
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                        BezierSplitFragment2::Materialized {
                            curve: BezierSubcurve2::Quadratic(_),
                            ..
                        }
                    ) | (
                        BezierSplitFragment2::Materialized {
                            curve: BezierSubcurve2::Quadratic(_),
                            ..
                        },
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                    )
                ));
                let result = region
                    .fillet_loop_vertex_by_radius(
                        0,
                        corner,
                        (Real::one() / Real::from(10_i8)).unwrap(),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the selected-circle/direct-Bezier corner must fillet exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert_eq!(result.certainty, CurveCertainty::Certified);
                let CurveCornerSolutions2::Unique(filleted) = result.value else {
                    panic!(
                        "the selected-circle/direct-Bezier corner must have one retained fillet: policy={policy:?}, reversed={reversed}"
                    );
                };
                assert_eq!(
                    filleted.boundary_loops()[0]
                        .fragments()
                        .iter()
                        .filter(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        ))
                        .count(),
                    2,
                );
                assert!(
                    filleted.boundary_loops()[0]
                        .fragments()
                        .iter()
                        .any(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicEndpointImages { start, end, .. }
                                if matches!(start, BezierParameter2::Algebraic(_))
                                    || matches!(end, BezierParameter2::Algebraic(_))
                        ))
                );
                if policy == CurveContext::STRICT && !reversed {
                    assert_eq!(
                        filleted
                            .classify_point(&p(0, 0), &policy)
                            .expect("the retained direct-Bezier fillet remains classifiable")
                            .into_value(),
                        Classification::Decided(RegionPointLocation::Inside),
                    );
                }
            }
        }
    }

    fn positive_inverse_sqrt_parameter(denominator: i8, policy: &CurveContext) -> BezierParameter2 {
        let polynomial = match BezierParameterPolynomial::try_new_power_basis(
            vec![-Real::one(), Real::zero(), Real::from(denominator)],
            policy,
        )
        .unwrap()
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => panic!("inverse-square polynomial: {reason:?}"),
        };
        let roots = match polynomial.isolate_unit_interval_roots(policy).unwrap() {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => panic!("inverse-square root: {reason:?}"),
        };
        let [parameter] = roots.as_slice() else {
            panic!("one positive inverse-square root must lie in the unit interval");
        };
        parameter.clone()
    }

    #[test]
    fn algebraic_query_winding_mixes_genuine_parallel_and_native_fragments() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let source = QuadraticBezier2::new(p(0, 0), Point2::new(half, Real::zero()), p(1, 1));
        let parallel = source.parallel_left(quarter).unwrap();
        let query_curve =
            RationalBezier2::try_new(vec![p(0, 0), p(0, 1)], vec![Real::one(), Real::one()])
                .unwrap();

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert!(matches!(
                parallel.exact_rational_parallel_component(&policy).unwrap(),
                Classification::Decided(None)
            ));
            let parameter = sqrt_half_algebraic_parameter(&policy);
            let BezierParameter2::Algebraic(parameter) = parameter else {
                panic!("sqrt(1/2) must remain algebraic");
            };
            let query = query_curve
                .point_at_algebraic_parameter(&parameter, &policy)
                .unwrap();
            let Classification::Decided(query) = query.predicate_evaluator(&policy).unwrap() else {
                panic!("the algebraic query predicate must construct");
            };
            let zero = Real::zero();
            let one = Real::one();
            let Classification::Decided(start) = parallel.point_at(&zero, &policy).unwrap() else {
                panic!("the genuine parallel start must evaluate");
            };
            let Classification::Decided(end) = parallel.point_at(&one, &policy).unwrap() else {
                panic!("the genuine parallel end must evaluate");
            };
            let range = BezierParameterRange2::new_validated(
                BezierParameter2::Exact(zero.clone()),
                BezierParameter2::Exact(one.clone()),
            );
            let Classification::Decided(analytic) =
                crate::BezierParallelFragment2::try_new(parallel.clone(), range, &policy).unwrap()
            else {
                panic!("the genuine parallel fragment must construct");
            };
            let corner = p(2, 0);
            let closure = |start: Point2, end: Point2| BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(zero.clone()),
                end: BezierParameter2::Exact(one.clone()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(start, end).unwrap(),
                )),
            };
            let boundary = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::AnalyticParallel(analytic),
                    closure(end, corner.clone()),
                    closure(corner, start),
                ],
                &policy,
            )
            .unwrap();
            let Classification::Decided(fragments) =
                prepare_algebraic_ray_retained_fragments(&boundary, &policy).unwrap()
            else {
                panic!("the mixed algebraic ray fragments must prepare");
            };
            let [
                AlgebraicRayRetainedFragment2::AnalyticParallel(analytic),
                AlgebraicRayRetainedFragment2::Rational(first_closure),
                AlgebraicRayRetainedFragment2::Rational(second_closure),
            ] = fragments.as_slice()
            else {
                panic!("the mixed loop must retain analytic and rational evaluators");
            };
            assert_eq!(
                analytic.contains_point(&query, &policy).unwrap(),
                Classification::Decided(false),
            );
            assert_eq!(
                algebraic_point_on_rational_fragment(first_closure, &query, &policy).unwrap(),
                Classification::Decided(false),
            );
            assert_eq!(
                algebraic_point_on_rational_fragment(second_closure, &query, &policy).unwrap(),
                Classification::Decided(false),
            );
            assert_eq!(
                algebraic_ray_retained_fragments_admit_direction(
                    &fragments,
                    &query,
                    &Real::zero(),
                    &Real::one(),
                    &policy,
                )
                .unwrap(),
                Classification::Decided(true),
            );
            assert_eq!(
                algebraic_ray_retained_fragments_winding(
                    &fragments,
                    &query,
                    &Real::one(),
                    &Real::zero(),
                    None,
                    &policy,
                )
                .unwrap(),
                Classification::Decided(0),
            );
            assert_eq!(
                classify_algebraic_point_against_retained_loop(
                    &boundary,
                    &query,
                    FillRule::NonZero,
                    true,
                    &policy,
                )
                .unwrap(),
                Classification::Decided(ContourPointLocation::Outside),
            );
        }
    }

    #[test]
    fn algebraic_retained_range_ray_winding_handles_crossing_multiplicity() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let alpha = sqrt_half_algebraic_parameter(&policy);
            let BezierParameter2::Algebraic(alpha_root) = &alpha else {
                panic!("sqrt(1/2) must remain algebraic");
            };
            let half = (Real::one() / Real::from(2_i8)).unwrap();
            let eighth = (Real::one() / Real::from(8_i8)).unwrap();
            let query_curve = RationalBezier2::try_new(
                vec![
                    Point2::new(Real::zero(), half.clone()),
                    Point2::new(Real::one(), half.clone()),
                ],
                vec![Real::one(); 2],
            )
            .expect("valid query carrier");
            let query = query_curve
                .point_at_algebraic_parameter(alpha_root, &policy)
                .expect("selected query point");
            let query = match query.predicate_evaluator(&policy).unwrap() {
                Classification::Decided(query) => query,
                Classification::Uncertain(reason) => {
                    panic!("selected query predicate: {reason:?}")
                }
            };
            let range = BezierParameterRange2::new_validated(
                alpha.clone().unit_complement(),
                alpha.clone(),
            );
            let rational = |points: Vec<Point2>| {
                RationalBezier2::try_new(points.clone(), vec![Real::one(); points.len()])
                    .expect("valid polynomial rational carrier")
            };
            let x = Real::from(2_i8);
            let double = rational(vec![
                Point2::new(x.clone(), Real::from(6_i8) * &eighth),
                Point2::new(x.clone(), Real::from(2_i8) * &eighth),
                Point2::new(x.clone(), Real::from(6_i8) * &eighth),
            ]);
            let triple = rational(vec![
                Point2::new(x.clone(), Real::from(3_i8) * &eighth),
                Point2::new(x.clone(), Real::from(5_i8) * &eighth),
                Point2::new(x.clone(), Real::from(3_i8) * &eighth),
                Point2::new(x.clone(), Real::from(5_i8) * &eighth),
            ]);
            let outside = rational(vec![
                Point2::new(x.clone(), Real::from(2_i8) * &eighth),
                Point2::new(x, Real::from(10_i8) * &eighth),
            ]);
            let winding = |curve: RationalBezier2, reversed: bool| {
                let endpoints = [
                    RationalBezierIntersectionPointEvidence2::Exact(curve.start().clone()),
                    RationalBezierIntersectionPointEvidence2::Exact(curve.end().clone()),
                ];
                let fragment = AlgebraicRayRationalFragment2 {
                    curve,
                    retained_range: Some(CurveRegionParameterRange2::from_bezier_range(
                        range.clone(),
                    )),
                    reversed,
                    endpoints,
                };
                algebraic_point_rational_curve_ray_winding(
                    &fragment,
                    &query,
                    &Real::one(),
                    &Real::zero(),
                    &policy,
                )
                .expect("exact retained-range winding")
            };
            assert_eq!(winding(double, false), Classification::Decided(0));
            assert_eq!(winding(triple.clone(), false), Classification::Decided(1));
            assert_eq!(winding(triple, true), Classification::Decided(-1));
            assert_eq!(winding(outside, false), Classification::Decided(0));
        }
    }

    #[test]
    fn independent_field_algebraic_chord_closes_and_classifies_a_region() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let x_parameter = positive_inverse_sqrt_parameter(2, &policy);
            let y_parameter = positive_inverse_sqrt_parameter(3, &policy);
            let x_source = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap(),
            ));
            let y_source = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(p(0, 0), p(0, 1)).unwrap(),
            ));
            let endpoint_image = |source: &BezierSubcurve2, parameter: &BezierParameter2| {
                let BezierParameter2::Algebraic(parameter) = parameter else {
                    panic!("the selected endpoint must remain algebraic");
                };
                BezierAlgebraicEndpointImage2::from_source_curve(source, parameter, &policy)
                    .unwrap()
            };
            let x_fragment = BezierSplitFragment2::AlgebraicEndpointImages {
                reversed: false,
                start: BezierParameter2::Exact(Real::zero()),
                end: x_parameter.clone(),
                source_curve: Some(x_source.clone()),
                start_image: None,
                end_image: Some(endpoint_image(&x_source, &x_parameter)),
            };
            let y_fragment = BezierSplitFragment2::AlgebraicEndpointImages {
                reversed: true,
                start: BezierParameter2::Exact(Real::zero()),
                end: y_parameter.clone(),
                source_curve: Some(y_source.clone()),
                start_image: None,
                end_image: Some(endpoint_image(&y_source, &y_parameter)),
            };
            let point_evidence = |source: &BezierSubcurve2, parameter: &BezierParameter2| {
                let source = RationalBezier2::try_from_subcurve(source).unwrap();
                crate::rational_bezier_general::exact_contact_point_evidence(
                    &source, parameter, &policy,
                )
                .unwrap()
                .expect("the algebraic line endpoint must retain point evidence")
            };
            let chord = match crate::BezierAlgebraicChord2::try_new(
                point_evidence(&x_source, &x_parameter),
                point_evidence(&y_source, &y_parameter),
                &policy,
            )
            .unwrap()
            {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    panic!("independent algebraic chord: {reason:?}")
                }
            };
            let boundary = CurveRegionBoundaryLoop2::new(
                vec![
                    x_fragment,
                    BezierSplitFragment2::AlgebraicChord(chord),
                    y_fragment,
                ],
                &policy,
            )
            .expect("the retained triangle must close by exact endpoint evidence");
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![CurveBoundaryInteriorSide2::Left],
            )
            .unwrap();
            let tenth = (Real::one() / Real::from(10_i8)).unwrap();
            assert_eq!(
                region
                    .classify_point(&Point2::new(tenth.clone(), tenth), &policy)
                    .unwrap()
                    .into_value(),
                Classification::Decided(RegionPointLocation::Inside)
            );
            assert_eq!(
                region
                    .classify_point(&p(1, 1), &policy)
                    .unwrap()
                    .into_value(),
                Classification::Decided(RegionPointLocation::Outside)
            );
            assert_eq!(
                region
                    .classify_point(&p(0, 0), &policy)
                    .unwrap()
                    .into_value(),
                Classification::Decided(RegionPointLocation::Boundary)
            );
        }
    }

    #[test]
    fn retained_parallel_offset_composition_respects_traversal_orientation() {
        let policy = CurveContext::STRICT;
        let tenth = (Real::one() / Real::from(10_i8)).unwrap();
        let fifth = (Real::one() / Real::from(5_i8)).unwrap();
        let source = QuadraticBezier2::new(p(1, 0), p(1, 1), p(0, 1));
        let parallel = source.parallel_left(-tenth.clone()).unwrap();
        let range = BezierParameterRange2::new_validated(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        );

        for (reversed, next_distance, expected_distance) in [
            (false, -fifth.clone(), -Real::from(3_i8) * tenth.clone()),
            (true, fifth, -Real::from(3_i8) * tenth),
        ] {
            let fragment = crate::BezierParallelFragment2::from_certified_range(
                parallel.clone(),
                range.clone(),
                reversed,
            );
            let Classification::Decided(span) =
                exact_offset_span_from_analytic_parallel(&fragment, &next_distance, &policy)
                    .unwrap()
            else {
                panic!("regular retained parallel composition must be decided");
            };
            assert_eq!(span.fragments.len(), 1);
            let BezierSplitFragment2::AnalyticParallel(composed) = &span.fragments[0] else {
                panic!("a non-PH quadratic composition remains analytic");
            };
            assert_eq!(composed.is_reversed(), reversed);
            assert_eq!(composed.parallel().distance(), &expected_distance);
        }
    }

    #[test]
    fn retained_parallel_offset_composition_splits_new_cusps_in_traversal_order() {
        let policy = CurveContext::STRICT;
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let source =
            QuadraticBezier2::new(p(0, 0), Point2::new(half.clone(), Real::zero()), p(1, 1));
        let parallel = source.parallel_left(Real::zero()).unwrap();
        let range = BezierParameterRange2::new_validated(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        );
        let cusp_distance = Real::from(2_i8).sqrt().unwrap();

        for (reversed, distance) in [(false, cusp_distance.clone()), (true, -cusp_distance)] {
            let fragment = crate::BezierParallelFragment2::from_certified_range(
                parallel.clone(),
                range.clone(),
                reversed,
            );
            let Classification::Decided(span) =
                exact_offset_span_from_analytic_parallel(&fragment, &distance, &policy).unwrap()
            else {
                panic!("the represented composed cusp must split exactly");
            };
            assert_eq!(span.fragments.len(), 2);
            let composed = span
                .fragments
                .iter()
                .map(|fragment| {
                    let BezierSplitFragment2::AnalyticParallel(fragment) = fragment else {
                        panic!("a general quadratic cusp split remains analytic");
                    };
                    assert_eq!(fragment.is_reversed(), reversed);
                    fragment.range()
                })
                .collect::<Vec<_>>();
            if reversed {
                assert_eq!(composed[0].start(), &BezierParameter2::Exact(half.clone()));
                assert_eq!(composed[0].end(), &BezierParameter2::Exact(Real::one()));
                assert_eq!(composed[1].start(), &BezierParameter2::Exact(Real::zero()));
                assert_eq!(composed[1].end(), &BezierParameter2::Exact(half.clone()));
            } else {
                assert_eq!(composed[0].start(), &BezierParameter2::Exact(Real::zero()));
                assert_eq!(composed[0].end(), &BezierParameter2::Exact(half.clone()));
                assert_eq!(composed[1].start(), &BezierParameter2::Exact(half.clone()));
                assert_eq!(composed[1].end(), &BezierParameter2::Exact(Real::one()));
            }
        }
    }

    #[test]
    fn retained_parallel_offset_coalesces_non_cusp_algebraic_arrangement_partitions() {
        let construction_policy = CurveContext::STRICT;
        let algebraic = sqrt_half_algebraic_parameter(&construction_policy);
        let zero = BezierParameter2::Exact(Real::zero());
        let one = BezierParameter2::Exact(Real::one());
        let range = |start: BezierParameter2, end: BezierParameter2| {
            let range = BezierParameterRange2::try_new(start, end, &construction_policy)
                .expect("the retained parameter range is valid");
            let Classification::Decided(range) = range else {
                panic!("the isolated range ordering must be decided");
            };
            range
        };
        let parallel = QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0))
            .parallel_left(Real::zero())
            .expect("the source has an exact analytic parallel");
        let fragment = |range| {
            let fragment = crate::BezierParallelFragment2::try_new(
                parallel.clone(),
                range,
                &construction_policy,
            )
            .expect("the regular parallel range is valid");
            let Classification::Decided(fragment) = fragment else {
                panic!("the regular parallel range must be decided");
            };
            fragment
        };
        let first = fragment(range(zero.clone(), algebraic.clone()));
        let second = fragment(range(algebraic, one.clone()));
        assert!(matches!(
            exact_offset_span_from_analytic_parallel(
                &first,
                &(Real::one() / Real::from(10_i8)).unwrap(),
                &construction_policy,
            ),
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        ));

        let split_fragments = vec![
            BezierSplitFragment2::AnalyticParallel(first.clone()),
            BezierSplitFragment2::AnalyticParallel(second.clone()),
        ];
        let Classification::Decided(Some((coalesced, consumed))) =
            coalesced_retained_parallel_offset_run(
                &split_fragments,
                0,
                split_fragments.len(),
                &construction_policy,
            )
            .expect("the arrangement-only partition is exactly coalescible")
        else {
            panic!("the arrangement-only partition must coalesce");
        };
        assert_eq!(consumed, 2);
        assert!(!coalesced.is_reversed());
        assert_eq!(
            coalesced.range().exact_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );

        let reversed_split_fragments = vec![
            BezierSplitFragment2::AnalyticParallel(second.reversed()),
            BezierSplitFragment2::AnalyticParallel(first.reversed()),
        ];
        let Classification::Decided(Some((coalesced_reversed, consumed))) =
            coalesced_retained_parallel_offset_run(
                &reversed_split_fragments,
                0,
                reversed_split_fragments.len(),
                &construction_policy,
            )
            .expect("the reversed arrangement-only partition is exactly coalescible")
        else {
            panic!("the reversed arrangement-only partition must coalesce");
        };
        assert_eq!(consumed, 2);
        assert!(coalesced_reversed.is_reversed());
        assert_eq!(
            coalesced_reversed.range().exact_endpoints(),
            Some((&Real::zero(), &Real::one()))
        );

        let closed_loop =
            |mut fragments: Vec<BezierSplitFragment2>, reversed: bool, cyclic_seam: bool| {
                fragments.push(if reversed {
                    quadratic_fragment(p(0, 0), p(1, 0), p(2, 0))
                } else {
                    quadratic_fragment(p(2, 0), p(1, 0), p(0, 0))
                });
                if cyclic_seam {
                    fragments.rotate_left(1);
                    assert!(matches!(
                        fragments.first(),
                        Some(BezierSplitFragment2::AnalyticParallel(first))
                            if !analytic_parallel_traversal_start(first).is_exact()
                    ));
                }
                CurveRegion2::try_new_with_loop_topology(
                    vec![
                        CurveRegionBoundaryLoop2::new(fragments, &construction_policy)
                            .expect("the algebraic partition retains exact connectivity"),
                    ],
                    vec![CurveRegionLoopRole::Material],
                    vec![FillRule::NonZero],
                    vec![if reversed {
                        CurveBoundaryInteriorSide2::Left
                    } else {
                        CurveBoundaryInteriorSide2::Right
                    }],
                )
                .expect("the exact cap topology is authored")
            };
        let unsplit = fragment(range(zero, one));
        let region_pairs = [
            (
                closed_loop(split_fragments.clone(), false, false),
                closed_loop(
                    vec![BezierSplitFragment2::AnalyticParallel(unsplit.clone())],
                    false,
                    false,
                ),
            ),
            (
                closed_loop(split_fragments, false, true),
                closed_loop(
                    vec![BezierSplitFragment2::AnalyticParallel(unsplit.clone())],
                    false,
                    false,
                ),
            ),
            (
                closed_loop(reversed_split_fragments.clone(), true, false),
                closed_loop(
                    vec![BezierSplitFragment2::AnalyticParallel(unsplit.reversed())],
                    true,
                    false,
                ),
            ),
            (
                closed_loop(reversed_split_fragments, true, true),
                closed_loop(
                    vec![BezierSplitFragment2::AnalyticParallel(unsplit.reversed())],
                    true,
                    false,
                ),
            ),
        ];
        let distance = (Real::one() / Real::from(10_i8)).unwrap();
        for (split_region, unsplit_region) in region_pairs {
            let mut reference = None;
            for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                let split_offset = split_region
                    .offset(distance.clone(), &OffsetCornerStyle2::Round, &policy)
                    .expect("the algebraically partitioned exact offset must complete");
                let unsplit_offset = unsplit_region
                    .offset(distance.clone(), &OffsetCornerStyle2::Round, &policy)
                    .expect("the equivalent unsplit exact offset must complete");
                assert_eq!(split_offset.certainty, unsplit_offset.certainty);
                if policy == CurveContext::STRICT {
                    assert_eq!(split_offset.certainty, CurveCertainty::Certified);
                }
                assert_eq!(split_offset.value, unsplit_offset.value);
                if let Some(reference) = &reference {
                    assert_eq!(&split_offset.value, reference);
                } else {
                    reference = Some(split_offset.value);
                }
            }
        }
    }

    #[test]
    fn retained_parallel_offset_preserves_algebraic_cusp_partition() {
        let construction_policy = CurveContext::STRICT;
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let parallel = CubicBezier2::new(p(0, 0), p(0, 4), p(4, -4), p(4, 0))
            .parallel_left(half)
            .expect("the source has an exact analytic parallel");
        let analysis = parallel
            .singularity_analysis(&construction_policy)
            .expect("the parallel cusp analysis is valid");
        let Classification::Decided(analysis) = analysis else {
            panic!("the exact cusp analysis must be decided");
        };
        let [cusp, next_cusp] = analysis.parallel_cusps() else {
            panic!("the selected parallel must have two cusps");
        };
        assert!(!cusp.is_exact());
        let make_fragment = |start: BezierParameter2, end: BezierParameter2| {
            let range = BezierParameterRange2::try_new(start, end, &construction_policy)
                .expect("the cusp range is valid");
            let Classification::Decided(range) = range else {
                panic!("the cusp range ordering must be decided");
            };
            let fragment = crate::BezierParallelFragment2::try_new(
                parallel.clone(),
                range,
                &construction_policy,
            )
            .expect("a cusp is permitted at a retained range endpoint");
            let Classification::Decided(fragment) = fragment else {
                panic!("the cusp-bounded regular fragment must be decided");
            };
            fragment
        };
        let first = make_fragment(BezierParameter2::Exact(Real::zero()), cusp.clone());
        let second = make_fragment(cusp.clone(), next_cusp.clone());
        let fragments = vec![
            BezierSplitFragment2::AnalyticParallel(first.clone()),
            BezierSplitFragment2::AnalyticParallel(second.clone()),
        ];

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first_scale = first
                .parallel()
                .regular_fragment_derivative_scale_sign(first.range(), &policy)
                .expect("the first limiting branch scale is valid");
            let second_scale = second
                .parallel()
                .regular_fragment_derivative_scale_sign(second.range(), &policy)
                .expect("the second limiting branch scale is valid");
            let (Classification::Decided(first_scale), Classification::Decided(second_scale)) =
                (first_scale, second_scale)
            else {
                panic!("both cusp-side derivative scales must be decided");
            };
            assert_ne!(first_scale, second_scale);
            assert_eq!(
                coalesced_retained_parallel_offset_run(&fragments, 0, fragments.len(), &policy)
                    .expect("the cusp partition decision is exact"),
                Classification::Decided(None),
            );
        }
    }

    #[test]
    fn regularized_analytic_loop_roles_follow_exact_nesting() {
        fn analytic_loop(
            radius: i32,
            center_x: i32,
            source_base: usize,
            policy: &CurveContext,
        ) -> CurveRegionBoundaryLoop2 {
            let sources = [
                QuadraticBezier2::new(
                    p(center_x + radius, 0),
                    p(center_x + radius, radius),
                    p(center_x, radius),
                ),
                QuadraticBezier2::new(
                    p(center_x, radius),
                    p(center_x - radius, radius),
                    p(center_x - radius, 0),
                ),
                QuadraticBezier2::new(
                    p(center_x - radius, 0),
                    p(center_x - radius, -radius),
                    p(center_x, -radius),
                ),
                QuadraticBezier2::new(
                    p(center_x, -radius),
                    p(center_x + radius, -radius),
                    p(center_x + radius, 0),
                ),
            ];
            let range = BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            );
            let fragments = sources
                .into_iter()
                .map(|source| {
                    BezierSplitFragment2::AnalyticParallel(
                        crate::BezierParallelFragment2::from_certified_range(
                            source.parallel_left(Real::zero()).unwrap(),
                            range.clone(),
                            false,
                        ),
                    )
                })
                .collect::<Vec<_>>();
            CurveRegionBoundaryLoop2::try_new_with_arrangement_sources(
                fragments,
                (0..4)
                    .map(|index| {
                        CurveRegionFragmentSource2::new(source_base + index, source_base + index, 0)
                    })
                    .collect(),
                policy,
            )
            .unwrap()
        }

        let policy = CurveContext::STRICT;
        let nested = CurveRegion2::new(vec![
            analytic_loop(4, 0, 0, &policy),
            analytic_loop(2, 0, 4, &policy),
        ])
        .unwrap()
        .with_certified_regularized_filled_left_topology()
        .unwrap();
        assert_eq!(
            nested.loop_roles_raw(&policy),
            Ok(Classification::Decided(vec![
                CurveRegionLoopRole::Material,
                CurveRegionLoopRole::Hole,
            ]))
        );

        let disjoint = CurveRegion2::new(vec![
            analytic_loop(4, 0, 0, &policy),
            analytic_loop(2, 10, 4, &policy),
        ])
        .unwrap()
        .with_certified_regularized_filled_left_topology()
        .unwrap();
        assert_eq!(
            disjoint.loop_roles_raw(&policy),
            Ok(Classification::Decided(vec![
                CurveRegionLoopRole::Material,
                CurveRegionLoopRole::Material,
            ]))
        );
    }

    #[test]
    fn exact_line_fragment_lowering_rejects_nonlinear_algebraic_source() {
        let policy = CurveContext::STRICT;
        let polynomial = BezierParameterPolynomial::try_new_power_basis(
            vec![Real::from(-1_i8), Real::zero(), Real::from(2_i8)],
            &policy,
        )
        .expect("the quadratic parameter polynomial is valid");
        let Classification::Decided(polynomial) = polynomial else {
            panic!("the exact polynomial must be decided");
        };
        let interval = BezierParameterInterval::try_new(
            (Real::from(2_i8) / Real::from(3_i8)).unwrap(),
            (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
            &policy,
        )
        .expect("the isolating interval is valid");
        let Classification::Decided(interval) = interval else {
            panic!("the exact interval must be decided");
        };
        let parameter = BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
            .expect("sqrt(1/2) has one root in the supplied interval");
        let Classification::Decided(parameter) = parameter else {
            panic!("the exact algebraic parameter must be decided");
        };

        // In power form this is `(x, y) = (2t^2 - 1, 2t^3 - t)`.
        // Both coordinates are exactly zero at the irrational split while the
        // source image itself is not a line.
        let source = RationalBezier2::try_new(
            vec![
                Point2::new(Real::from(-1_i8), Real::zero()),
                Point2::new(
                    Real::from(-1_i8),
                    (Real::from(-1_i8) / Real::from(3_i8)).unwrap(),
                ),
                Point2::new(
                    (Real::from(-1_i8) / Real::from(3_i8)).unwrap(),
                    (Real::from(-2_i8) / Real::from(3_i8)).unwrap(),
                ),
                Point2::new(Real::one(), Real::one()),
            ],
            vec![Real::one(); 4],
        )
        .expect("the polynomial cubic has a rational Bezier representation");
        let split = source
            .split_at_parameters(&[BezierParameter2::algebraic(parameter)], &policy)
            .expect("the exact algebraic split is constructible");
        let Classification::Decided(split) = split else {
            panic!("the exact algebraic split must be decided");
        };
        assert!(matches!(
            retained_line_fragment_segment(&split.fragments()[0], &policy),
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        ));
    }

    #[test]
    fn curve_region_is_one_word_and_empty_data_is_process_shared() {
        assert!(
            core::mem::size_of::<PolicyEvaluationCache<Option<Real>>>()
                <= core::mem::size_of::<OnceLock<CurveResult<Option<Real>>>>()
                    + core::mem::size_of::<usize>(),
            "policy-aware signed-area caching must add at most one alignment word"
        );
        let first = CurveRegion2::empty();
        let second = CurveRegion2::default();

        assert_eq!(
            core::mem::size_of::<CurveRegion2>(),
            core::mem::size_of::<usize>()
        );
        assert!(Arc::ptr_eq(&first.data, &second.data));
        assert!(first.clone().into_boundary_loops().is_empty());
    }

    #[test]
    fn boundary_path_construction_obeys_selected_terminal_policy() {
        let start = Point2::new(Real::pi() + Real::e(), Real::zero());
        let end = Point2::new(Real::e() + Real::pi(), Real::zero());
        let path = CurvePath2::try_new(vec![Curve2::from(QuadraticBezier2::new(
            start,
            p(0, 1),
            end,
        ))])
        .expect("one-curve path construction has no adjacency decision");

        let strict = CurveRegion2::try_from_boundary_paths(
            std::slice::from_ref(&path),
            &CurveContext::STRICT,
        )
        .expect_err("strict construction must preserve an undecidable closure");
        assert!(matches!(
            strict,
            ExactCurveError::Blocked(blocker)
                if blocker.operation() == CurveOperation2::Construction
                    && blocker.reason() == UncertaintyReason::RealSign
        ));

        let approximate = CurveRegion2::try_from_boundary_paths(
            std::slice::from_ref(&path),
            &CurveContext::APPROXIMATE_512,
        )
        .expect("the authorized terminal equality must close the path");
        assert_eq!(
            approximate.certainty,
            CurveCertainty::Approximate512Consumed
        );
        assert_eq!(approximate.value.len(), 1);
        assert!(
            !approximate
                .value
                .data
                .strict_materialized_connectivity_certified
        );

        let exact_start = p(0, 0);
        let exact_path = CurvePath2::try_new(vec![Curve2::from(QuadraticBezier2::new(
            exact_start.clone(),
            p(1, 1),
            exact_start,
        ))])
        .unwrap();
        let exact = CurveRegion2::try_from_boundary_paths(&[exact_path], &CurveContext::STRICT)
            .unwrap()
            .into_value();
        assert!(exact.data.strict_materialized_connectivity_certified);
    }

    #[test]
    fn rational_line_measurements_obey_policy_and_isolate_cached_certainty() {
        let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
        let line_y = || Real::one() + &undecidable_zero;
        let rational = RationalBezier2::try_new(
            vec![
                p(1, 1),
                Point2::new(Real::from(2_i8), line_y()),
                Point2::new(Real::from(3_i8), line_y()),
                Point2::new(Real::from(4_i8), line_y()),
                p(5, 1),
            ],
            vec![
                Real::one(),
                Real::from(2_i8),
                Real::from(3_i8),
                Real::from(5_i8),
                Real::from(10_i8),
            ],
        )
        .expect("positive weights define a finite rational curve");
        assert_eq!(rational.signed_area_contribution().unwrap(), None);
        assert_eq!(rational.area_moments_contribution().unwrap(), None);

        let curve = BezierSubcurve2::Rational(rational);
        let strict_area = curve
            .signed_area_contribution(&CurveContext::STRICT)
            .unwrap();
        assert_eq!(strict_area.certainty, CurveCertainty::Certified);
        assert_eq!(
            strict_area.value,
            Classification::Uncertain(UncertaintyReason::RealSign)
        );
        let approximate_area = curve
            .signed_area_contribution(&CurveContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(
            approximate_area.certainty,
            CurveCertainty::Approximate512Consumed
        );
        assert_eq!(
            approximate_area.value,
            Classification::Decided(Some(Real::from(-2_i8)))
        );

        let strict_moments = curve
            .area_moments_contribution(&CurveContext::STRICT)
            .unwrap();
        assert_eq!(strict_moments.certainty, CurveCertainty::Certified);
        assert_eq!(
            strict_moments.value,
            Classification::Uncertain(UncertaintyReason::RealSign)
        );
        let expected_moments = BezierAreaMoments2::line_contribution(&p(1, 1), &p(5, 1)).unwrap();
        let approximate_moments = curve
            .area_moments_contribution(&CurveContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(
            approximate_moments.certainty,
            CurveCertainty::Approximate512Consumed
        );
        assert_eq!(
            approximate_moments.value,
            Classification::Decided(Some(expected_moments))
        );

        let loop_ = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve,
                },
                quadratic_fragment(p(5, 1), p(5, 2), p(5, 3)),
                quadratic_fragment(p(5, 3), p(3, 3), p(1, 3)),
                quadratic_fragment(p(1, 3), p(1, 2), p(1, 1)),
            ],
            &CurveContext::STRICT,
        )
        .expect("exact endpoints close the retained loop");
        let region = CurveRegion2::new(vec![loop_]).expect("one retained loop");

        let approximate = region.signed_area(&CurveContext::APPROXIMATE_512).unwrap();
        assert_eq!(
            approximate.certainty,
            CurveCertainty::Approximate512Consumed
        );
        assert_eq!(
            approximate.value,
            Classification::Decided(Some(Real::from(8_i8)))
        );
        let strict = region.signed_area(&CurveContext::STRICT).unwrap();
        assert_eq!(strict.certainty, CurveCertainty::Certified);
        assert_eq!(
            strict.value,
            Classification::Uncertain(UncertaintyReason::RealSign)
        );
        assert_eq!(
            region
                .signed_area(&CurveContext::APPROXIMATE_512)
                .unwrap()
                .certainty,
            CurveCertainty::Approximate512Consumed
        );
    }

    #[test]
    fn curve_region_mutations_report_selected_terminal_policy() {
        let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
        let region = single_quadratic_loop_region(false);

        let scale = Real::one() + &undecidable_zero;
        let strict_transform = region
            .transform_affine(
                &scale,
                &Real::zero(),
                &Real::zero(),
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &CurveContext::STRICT,
            )
            .expect("the determinant is certified positive without deciding the symbolic zero");
        assert_eq!(strict_transform.certainty, CurveCertainty::Certified);
        let approximate_transform = region
            .transform_affine(
                &scale,
                &Real::zero(),
                &Real::zero(),
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &CurveContext::APPROXIMATE_512,
            )
            .expect("the same certified determinant is valid under the broader policy");
        assert_eq!(approximate_transform.certainty, CurveCertainty::Certified);
        let BezierSplitFragment2::Materialized {
            curve: BezierSubcurve2::Quadratic(first),
            ..
        } = &approximate_transform.value.boundary_loops()[0].fragments()[0]
        else {
            panic!("the transformed quadratic boundary must remain materialized");
        };
        let expected_control = affine_region_point(
            &p(1, 0),
            &scale,
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
        );
        assert_eq!(
            first.control(),
            &expected_control,
            "the terminal policy must not replace transformed coordinates"
        );

        let identity = crate::Similarity2::try_from_real_affine(
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::zero(),
            Real::zero(),
        )
        .unwrap();
        assert_eq!(
            region
                .transform_similarity(&identity, &CurveContext::APPROXIMATE_512)
                .unwrap()
                .certainty,
            CurveCertainty::Certified
        );

        let chamfer = region
            .chamfer_loop_vertex_by_parameters(
                0,
                0,
                (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
                (Real::one() / Real::from(4_i8)).unwrap(),
                &CurveContext::APPROXIMATE_512,
            )
            .unwrap();
        assert_eq!(chamfer.certainty, CurveCertainty::Certified);
        assert!(matches!(chamfer.value, Classification::Decided(_)));

        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let symbolic_center = Point2::new(&half + &undecidable_zero, half);
        let strict_fillet = region
            .fillet_loop_vertex_by_parameters(
                0,
                0,
                (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
                (Real::one() / Real::from(4_i8)).unwrap(),
                &symbolic_center,
                false,
                &CurveContext::STRICT,
            )
            .unwrap();
        assert_eq!(strict_fillet.certainty, CurveCertainty::Certified);
        assert!(matches!(
            strict_fillet.value,
            Classification::Uncertain(UncertaintyReason::RealSign)
        ));
        let approximate_fillet = region
            .fillet_loop_vertex_by_parameters(
                0,
                0,
                (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
                (Real::one() / Real::from(4_i8)).unwrap(),
                &symbolic_center,
                false,
                &CurveContext::APPROXIMATE_512,
            )
            .unwrap();
        assert_eq!(
            approximate_fillet.certainty,
            CurveCertainty::Approximate512Consumed
        );
        assert!(matches!(
            &approximate_fillet.value,
            Classification::Decided(_)
        ));
        let Classification::Decided(approximate_fillet_region) = &approximate_fillet.value else {
            unreachable!("the approximate fillet was decided above");
        };
        let Classification::Decided(native_fillet) = approximate_fillet_region
            .native_line_arc_region(&CurveContext::APPROXIMATE_512)
            .unwrap()
        else {
            panic!("the native fillet must retain its line/arc accelerator");
        };
        assert_eq!(
            native_fillet.material_contours()[0].signed_area().unwrap(),
            None,
            "an unresolved center-arc quadrant must return unsupported area, not panic"
        );

        let bent = CurveRegion2::new(vec![
            CurveRegionBoundaryLoop2::new(
                vec![
                    quadratic_fragment(
                        p(0, 0),
                        Point2::new(Real::one(), Real::one() + &undecidable_zero),
                        p(2, 0),
                    ),
                    quadratic_fragment(p(2, 0), p(2, 1), p(2, 2)),
                    quadratic_fragment(p(2, 2), p(1, 2), p(0, 2)),
                    quadratic_fragment(p(0, 2), p(0, 1), p(0, 0)),
                ],
                &CurveContext::STRICT,
            )
            .unwrap(),
        ])
        .unwrap();
        let flattening =
            BezierFlatteningOptions::try_new(Real::one(), 4, &CurveContext::STRICT).unwrap();
        let strict_segmentation = bent
            .segment_certified(&flattening, &CurveContext::STRICT)
            .unwrap();
        assert_eq!(strict_segmentation.certainty, CurveCertainty::Certified);
        assert!(matches!(
            strict_segmentation.value,
            Classification::Uncertain(UncertaintyReason::Ordering)
        ));
        let approximate_segmentation = bent
            .segment_certified(&flattening, &CurveContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(
            approximate_segmentation.certainty,
            CurveCertainty::Approximate512Consumed
        );
        assert!(matches!(
            approximate_segmentation.value,
            Classification::Decided(_)
        ));

        let collapse_distance = -Real::one() + &undecidable_zero;
        let strict_segmented_offset = region
            .offset_with_certified_segmentation(
                collapse_distance.clone(),
                &OffsetCornerStyle2::Round,
                &flattening,
                &CurveContext::STRICT,
            )
            .unwrap();
        assert_eq!(strict_segmented_offset.certainty, CurveCertainty::Certified);
        assert!(matches!(
            strict_segmented_offset.value,
            Classification::Uncertain(_)
        ));
        let approximate_segmented_offset = region
            .offset_with_certified_segmentation(
                collapse_distance.clone(),
                &OffsetCornerStyle2::Round,
                &flattening,
                &CurveContext::APPROXIMATE_512,
            )
            .unwrap();
        assert_eq!(
            approximate_segmented_offset.certainty,
            CurveCertainty::Approximate512Consumed
        );
        assert!(matches!(
            approximate_segmented_offset.value,
            Classification::Decided(_)
        ));

        let parallel =
            BezierParallelVerificationOptions::try_new(Real::one(), 4, &CurveContext::STRICT)
                .unwrap();
        let approximate_parallel_offset = region
            .offset_with_certified_bezier_parallel(
                collapse_distance,
                &OffsetCornerStyle2::Round,
                &parallel,
                &flattening,
                &flattening,
                &CurveContext::APPROXIMATE_512,
            )
            .unwrap();
        assert_eq!(
            approximate_parallel_offset.certainty,
            CurveCertainty::Approximate512Consumed
        );
        assert!(matches!(
            approximate_parallel_offset.value,
            Classification::Decided(_)
        ));
    }

    fn quadratic_fragment(start: Point2, control: Point2, end: Point2) -> BezierSplitFragment2 {
        BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(start, control, end)),
        }
    }

    fn rational_quadratic_fragment(
        start: Point2,
        control: Point2,
        end: Point2,
    ) -> BezierSplitFragment2 {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::RationalQuadratic(
                RationalQuadraticBezier2::try_new(
                    start,
                    control,
                    end,
                    Real::one(),
                    half,
                    Real::one(),
                )
                .unwrap(),
            ),
        }
    }

    fn single_rational_quadratic_loop_region() -> CurveRegion2 {
        let fragments = vec![
            rational_quadratic_fragment(p(0, 0), p(1, 0), p(2, 0)),
            rational_quadratic_fragment(p(2, 0), p(2, 1), p(2, 2)),
            rational_quadratic_fragment(p(2, 2), p(1, 2), p(0, 2)),
            rational_quadratic_fragment(p(0, 2), p(0, 1), p(0, 0)),
        ];
        CurveRegion2::new(vec![
            CurveRegionBoundaryLoop2::new(fragments, &CurveContext::STRICT)
                .expect("closed retained rational-quadratic loop"),
        ])
        .expect("one retained rational-quadratic loop")
    }

    fn single_quadratic_loop_region(clockwise: bool) -> CurveRegion2 {
        let fragments = if clockwise {
            vec![
                quadratic_fragment(p(0, 0), p(0, 1), p(0, 2)),
                quadratic_fragment(p(0, 2), p(1, 2), p(2, 2)),
                quadratic_fragment(p(2, 2), p(2, 1), p(2, 0)),
                quadratic_fragment(p(2, 0), p(1, 0), p(0, 0)),
            ]
        } else {
            vec![
                quadratic_fragment(p(0, 0), p(1, 0), p(2, 0)),
                quadratic_fragment(p(2, 0), p(2, 1), p(2, 2)),
                quadratic_fragment(p(2, 2), p(1, 2), p(0, 2)),
                quadratic_fragment(p(0, 2), p(0, 1), p(0, 0)),
            ]
        };
        CurveRegion2::new(vec![
            CurveRegionBoundaryLoop2::new(fragments, &CurveContext::STRICT)
                .expect("closed retained quadratic loop"),
        ])
        .expect("one retained loop")
    }

    #[test]
    fn curve_region_clones_share_geometry_and_lazy_caches() {
        let region = single_quadratic_loop_region(false);
        let clone = region.clone();
        let policy = CurveContext::STRICT;

        assert!(Arc::ptr_eq(&region.data, &clone.data));
        assert!(region.data.signed_area_cache.is_empty());
        let clone_area = clone.signed_area(&policy).expect("clone area").into_value();
        assert!(matches!(clone_area, Classification::Decided(Some(_))));
        assert!(!region.data.signed_area_cache.is_empty());
        assert_eq!(
            clone_area,
            region
                .signed_area(&policy)
                .expect("source area")
                .into_value()
        );

        let cloned_loops = clone.into_boundary_loops();
        assert_eq!(cloned_loops, region.boundary_loops());
        assert_eq!(region.len(), 1);
    }

    #[test]
    fn single_loop_filled_side_uses_area_without_constructing_nesting_bounds() {
        let policy = CurveContext::STRICT;
        for (clockwise, expected) in [(false, true), (true, false)] {
            let region = single_quadratic_loop_region(clockwise);
            assert!(region.data.native_boundary_bounds.is_empty());
            assert!(matches!(
                region.filled_side_is_left(&policy),
                Ok(CurveOutcome {
                    value: Classification::Decided(sides),
                    ..
                }) if sides == [expected]
            ));
            assert!(region.data.native_boundary_bounds.is_empty());
        }
    }

    #[test]
    fn native_query_bounds_use_exact_conservative_control_hulls() {
        let policy = CurveContext::STRICT;
        let cubic = CubicBezier2::new(p(0, 0), p(0, 6), p(4, 6), p(4, 0));
        let curve = BezierSubcurve2::Cubic(cubic.clone());
        let query_bounds = match subcurve_query_bounds(&curve, &policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                panic!("polynomial control hull unexpectedly uncertain: {reason:?}")
            }
        };
        let control_hull = match Aabb2::from_points(cubic.control_points(), &policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                panic!("polynomial control hull unexpectedly uncertain: {reason:?}")
            }
        };
        let tight_bounds = match cubic.certified_bounds(&policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                panic!("cubic tight bounds unexpectedly uncertain: {reason:?}")
            }
        };

        assert_eq!(query_bounds, control_hull);
        assert_eq!(
            compare_reals(query_bounds.max().y(), tight_bounds.max().y(), &policy),
            Some(Ordering::Greater)
        );
        for numerator in 0_i32..=8 {
            let parameter = (Real::from(numerator) / Real::from(8_i32)).unwrap();
            assert_eq!(
                query_bounds.contains_point(&cubic.point_at(parameter), &policy),
                Classification::Decided(true)
            );
        }
    }

    #[test]
    fn independent_region_orientations_share_equal_conic_area_kernels() {
        let policy = CurveContext::STRICT;
        let first = single_rational_quadratic_loop_region();
        let second = single_rational_quadratic_loop_region();
        let mut cache = RationalQuadraticAreaIntegralCache::default();

        assert!(matches!(
            first.filled_side_is_left_with_area_cache(&policy, &mut cache),
            Ok(Classification::Decided([true]))
        ));
        assert_eq!(cache.retained_integral_count(), 1);
        assert!(matches!(
            second.filled_side_is_left_with_area_cache(&policy, &mut cache),
            Ok(Classification::Decided([true]))
        ));
        assert_eq!(cache.retained_integral_count(), 1);
    }

    #[test]
    fn retained_subcurve_point_query_preserves_projective_denominator_uncertainty() {
        let conic = RationalQuadraticBezier2::try_new(
            p(0, 0),
            p(1, 0),
            p(2, 0),
            1.into(),
            (-1).into(),
            1.into(),
        )
        .unwrap();
        let subcurve = BezierSubcurve2::RationalQuadratic(conic);

        assert_eq!(
            subcurve_contains_point(&subcurve, &p(100, 0), &CurveContext::STRICT),
            Classification::Uncertain(UncertaintyReason::Boundary)
        );
    }

    #[test]
    fn irrational_weight_semicircle_region_recovers_exact_native_accelerator() {
        let policy = CurveContext::STRICT;
        let arcs = [
            CircularArc2::try_from_center(p(0, 0), p(2, 0), p(1, 0), true).unwrap(),
            CircularArc2::try_from_center(p(2, 0), p(0, 0), p(1, 0), true).unwrap(),
        ];
        let mut curves = Vec::with_capacity(4);
        for arc in arcs {
            for span in arc
                .rational_bezier_decomposition(&policy)
                .unwrap()
                .into_value()
                .spans()
            {
                let curve = span.curve();
                let controls = curve.control_points();
                let weights = curve.weights();
                curves.push(Curve2::from(
                    RationalQuadraticBezier2::try_new(
                        controls[0].clone(),
                        controls[1].clone(),
                        controls[2].clone(),
                        weights[0].clone(),
                        weights[1].clone(),
                        weights[2].clone(),
                    )
                    .unwrap(),
                ));
            }
        }
        let region =
            CurveRegion2::try_from_boundary_paths(&[CurvePath2::try_new(curves).unwrap()], &policy)
                .unwrap()
                .into_value();
        let point = Point2::new(Real::one(), (Real::one() / Real::from(2_u8)).unwrap());
        assert_eq!(
            region
                .classify_point(&point, &policy)
                .map(CurveOutcome::into_value),
            Ok(Classification::Decided(RegionPointLocation::Inside))
        );
        assert_eq!(
            region
                .classify_point(&p(1, 1), &policy)
                .map(CurveOutcome::into_value),
            Ok(Classification::Decided(RegionPointLocation::Boundary))
        );
        assert_eq!(
            region
                .classify_point(&p(1, 2), &policy)
                .map(CurveOutcome::into_value),
            Ok(Classification::Decided(RegionPointLocation::Outside))
        );
        assert!(matches!(
            region.data.line_image_region.certified(),
            Some(Some(_))
        ));
    }

    #[test]
    fn nonuniform_rational_line_images_use_exact_geometric_moments() {
        let expected = BezierAreaMoments2::line_contribution(&p(2, 0), &p(4, 2)).unwrap();
        let quadratic = BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(
                p(2, 0),
                p(3, 1),
                p(4, 2),
                Real::one(),
                Real::from(2),
                Real::from(3),
            )
            .unwrap(),
        );
        assert_eq!(
            quadratic
                .area_moments_contribution(&CurveContext::STRICT)
                .unwrap()
                .into_value(),
            Classification::Decided(Some(expected.clone()))
        );

        let rational = BezierSubcurve2::Rational(
            RationalBezier2::try_new(
                vec![p(2, 0), p(3, 1), p(4, 2)],
                vec![Real::one(), Real::from(3), Real::from(5)],
            )
            .unwrap(),
        );
        assert_eq!(
            rational
                .area_moments_contribution(&CurveContext::STRICT)
                .unwrap()
                .into_value(),
            Classification::Decided(Some(expected))
        );
    }

    #[test]
    fn explicit_signed_loops_classify_without_regularized_native_fast_path() {
        fn rectangle(min_x: i32, max_x: i32) -> CurvePath2 {
            let corners = [p(min_x, -3), p(max_x, -3), p(max_x, 3), p(min_x, 3)];
            CurvePath2::try_new(
                (0..4)
                    .map(|index| {
                        Curve2::from(
                            LineSeg2::try_new(
                                corners[index].clone(),
                                corners[(index + 1) % 4].clone(),
                            )
                            .unwrap(),
                        )
                    })
                    .collect(),
            )
            .unwrap()
        }

        let policy = CurveContext::STRICT;
        let region = CurveRegion2::try_from_signed_boundary_paths_with_loop_semantics(
            &[rectangle(-3, 3), rectangle(1, 7)],
            &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole],
            &[FillRule::NonZero, FillRule::NonZero],
            &policy,
        )
        .unwrap()
        .into_value();
        assert_eq!(
            region
                .classify_point(&p(-2, 0), &policy)
                .map(CurveOutcome::into_value),
            Ok(Classification::Decided(RegionPointLocation::Inside))
        );
        assert_eq!(
            region
                .classify_point(&p(2, 0), &policy)
                .map(CurveOutcome::into_value),
            Ok(Classification::Decided(RegionPointLocation::Outside))
        );
        assert_eq!(
            region
                .signed_depth(&p(2, 0), &policy)
                .map(CurveOutcome::into_value),
            Ok(Classification::Decided(0))
        );
    }
}
