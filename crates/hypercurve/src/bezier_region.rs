//! Exact curved regions bounded by native and algebraic curve fragments.
//!
//! [`CurveRegion2`] is the top-level higher-order region type. It accepts
//! closed [`CurvePath2`] boundaries directly and materializes decided Boolean
//! traversals without flattening their native or algebraic carriers. It
//! deliberately does not force curved boundaries into line strings or into
//! [`LineArcRegion2`](crate::LineArcRegion2), because the exactness model's exact geometric-computation
//! model requires the exact curve objects to remain visible until a certified
//! adapter exists.
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

use crate::bezier_arrangement::represented_roots_equal;
use crate::bezier_moment::RationalQuadraticAreaIntegralCache;
use crate::bezier_topology::exact_polynomial_line_contact_relation_from_direction;
use crate::classify::{compare_reals, is_zero, real_sign};
use crate::region_nesting::ExactCurveWorkspace2;
use crate::{
    Aabb2, Axis2, BezierAlgebraicEndpointImage2, BezierArrangementGraph2,
    BezierArrangementTraversal2, BezierEndpointPointImage2, BezierFlatteningOptions,
    BezierLineContact, BezierLineContactKind, BezierLineContactRelation,
    BezierLineCrossingDirection, BezierLineImageFitRelation, BezierParallelVerificationOptions,
    BezierParameter2, BezierRetainedLinearOverlapTraversal2,
    BezierRetainedRationalOverlapTraversal2, BezierSplitFragment2, BezierSubcurve2, BooleanOp,
    Classification, Contour2, ContourPointLocation, CubicBezier2, Curve2,
    CurveBoundaryInteriorSide2, CurveError, CurveFamily2, CurveGeometry2,
    CurveIntersectionPairBlockerKind2, CurveOperation2, CurvePath2, CurvePathIntersectionContact2,
    CurvePolicy, CurveResult, ExactCurveError, ExactCurveResult, FillRule, LineArcRegion2,
    LineSeg2, Point2, QuadraticBezier2, RationalBezier2, RationalBezierPointIncidence2,
    RationalQuadraticBezier2, RegionArrangement2, RegionArrangementSummary2, RegionPointLocation,
    RetainedTopologyStatus, Segment2, UncertaintyReason,
};

/// A closed native Bezier/conic boundary loop.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBoundaryLoop2 {
    fragments: Vec<BezierSubcurve2>,
}

/// A retained higher-order region with native Bezier/conic boundary loops.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BezierRegion2 {
    boundary_loops: Vec<BezierBoundaryLoop2>,
}

/// A closed retained Bezier/conic boundary loop.
///
/// Unlike [`BezierBoundaryLoop2`], this carrier may contain
/// [`BezierSplitFragment2::AlgebraicEndpointImages`] fragments.  It is a
/// concrete exact-object region boundary in the exactness model's sense: the algebraic pieces
/// remain replayable construction evidence, not sampled coordinates.
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
    boundary_loops: Vec<CurveRegionBoundaryLoop2>,
    certified_loop_roles: Option<Arc<[CurveRegionLoopRole]>>,
    certified_loop_fill_rules: Option<Arc<[FillRule]>>,
    signed_loop_composition: bool,
    filled_side_is_left: Arc<OnceLock<CurveResult<Classification<Arc<[bool]>>>>>,
    native_boundary_loops: Arc<OnceLock<Option<Arc<[BezierBoundaryLoop2]>>>>,
    native_boundary_bounds: Arc<OnceLock<Arc<[Aabb2]>>>,
    line_image_region: Arc<OnceLock<Option<LineArcRegion2>>>,
    retained_rational_evaluators: Arc<OnceLock<CurveResult<Vec<Vec<Option<RationalBezier2>>>>>>,
    signed_area_cache: Arc<OnceLock<CurveResult<Option<Real>>>>,
}

impl Default for CurveRegion2 {
    fn default() -> Self {
        let region = Self {
            boundary_loops: Vec::new(),
            certified_loop_roles: Some(Arc::from(Vec::new())),
            certified_loop_fill_rules: Some(Arc::from(Vec::new())),
            signed_loop_composition: false,
            filled_side_is_left: Arc::new(OnceLock::new()),
            native_boundary_loops: Arc::new(OnceLock::new()),
            native_boundary_bounds: Arc::new(OnceLock::new()),
            line_image_region: Arc::new(OnceLock::new()),
            retained_rational_evaluators: Arc::new(OnceLock::new()),
            signed_area_cache: Arc::new(OnceLock::new()),
        };
        let _ = region
            .filled_side_is_left
            .set(Ok(Classification::Decided(Arc::from(Vec::new()))));
        let _ = region.line_image_region.set(Some(LineArcRegion2::empty()));
        region
    }
}

/// Borrowed native line/arc contour acceleration view for a [`CurveRegion2`].
///
/// This exposes the useful fast-path geometry without transferring ownership
/// to the legacy [`LineArcRegion2`] container. Higher-order regions simply do not
/// produce this view.
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

/// Immediate native line/arc arrangement with a unified curved output.
#[derive(Clone, Debug)]
pub struct CurveRegionArrangement2 {
    region: Option<CurveRegion2>,
    workspace: Arc<ExactCurveWorkspace2>,
    summary: RegionArrangementSummary2,
}

/// Evidence-bearing native contour nesting with authoritative unified output.
///
/// The retained evidence is produced by the specialized line/arc nesting engine,
/// while successful topology is promoted before it crosses this API boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionBoundaryContourBuildResult2 {
    region: Option<CurveRegion2>,
    evidence: crate::RegionBoundaryContourBuildEvidence2,
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
    used_exact_native_fast_path: bool,
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
    used_exact_native_fast_path: bool,
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
    fn facts(&self) -> &ExactCurveWorkspace2 {
        &self.workspace
    }

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
    pub fn fill_rule(&self) -> FillRule {
        self.facts().request().fill_rule()
    }

    /// Returns the number of exact source segments evaluated by the arrangement.
    pub fn source_segment_count(&self) -> usize {
        self.facts().request().source_segment_count()
    }

    /// Returns final semantic facts from the completed arrangement.
    pub const fn summary(&self) -> &RegionArrangementSummary2 {
        &self.summary
    }

    /// Returns the final retained topology status, when evaluation completed.
    pub const fn status(&self) -> Option<RetainedTopologyStatus> {
        self.summary.status()
    }

    /// Returns the blocker when the completed arrangement could not materialize a region.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.summary.blocker()
    }

    /// Consumes the result and returns its unified region, if materialized.
    pub fn into_region(self) -> Option<CurveRegion2> {
        self.region
    }
}

impl CurveRegionBoundaryContourBuildResult2 {
    /// Returns the unified region when exact native nesting succeeded.
    pub const fn region(&self) -> Option<&CurveRegion2> {
        self.region.as_ref()
    }

    /// Returns the retained native nesting and role-assignment evidence.
    pub const fn evidence(&self) -> &crate::RegionBoundaryContourBuildEvidence2 {
        &self.evidence
    }

    /// Returns the retained native construction status.
    pub const fn status(&self) -> crate::RetainedTopologyStatus {
        self.evidence.status()
    }

    /// Returns the exact blocker when no unified region was materialized.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.evidence.blocker()
    }

    /// Returns the unified result as a classification without consuming evidence.
    pub fn region_classification(&self) -> Classification<&CurveRegion2> {
        match self.region() {
            Some(region) => Classification::Decided(region),
            None => Classification::Uncertain(
                self.evidence
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes the result and returns its unified region, if materialized.
    pub fn into_region(self) -> Option<CurveRegion2> {
        self.region
    }

    /// Consumes the result and returns its retained evidence.
    pub fn into_evidence(self) -> crate::RegionBoundaryContourBuildEvidence2 {
        self.evidence
    }

    /// Consumes the unified output and retained native evidence together.
    pub fn into_parts(
        self,
    ) -> (
        Option<CurveRegion2>,
        crate::RegionBoundaryContourBuildEvidence2,
    ) {
        (self.region, self.evidence)
    }

    /// Consumes the unified output as a classification.
    pub fn into_region_classification(self) -> Classification<CurveRegion2> {
        let blocker = self
            .evidence
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.region {
            Some(region) => Classification::Decided(region),
            None => Classification::Uncertain(blocker),
        }
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
    /// Returns true when no segmentation was needed because native offsetting succeeded exactly.
    pub const fn used_exact_native_fast_path(&self) -> bool {
        self.used_exact_native_fast_path
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
    /// Returns whether the exact native line/arc offset kernel completed the operation.
    pub const fn used_exact_native_fast_path(&self) -> bool {
        self.used_exact_native_fast_path
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
            .field("boundary_loops", &self.boundary_loops)
            .field("certified_loop_roles", &self.certified_loop_roles)
            .field("certified_loop_fill_rules", &self.certified_loop_fill_rules)
            .field("signed_loop_composition", &self.signed_loop_composition)
            .finish()
    }
}

impl PartialEq for CurveRegion2 {
    fn eq(&self, other: &Self) -> bool {
        self.boundary_loops == other.boundary_loops
            && self.certified_loop_roles == other.certified_loop_roles
            && self.certified_loop_fill_rules == other.certified_loop_fill_rules
            && self.signed_loop_composition == other.signed_loop_composition
    }
}

/// Material/hole role assigned to one retained Bezier boundary loop.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveRegionLoopRole {
    /// The loop contributes filled material.
    Material,
    /// The loop subtracts from the containing material loop.
    Hole,
}

/// One exact retained material boundary and the hole boundaries it owns.
///
/// This is the mixed-family counterpart to [`crate::RegionContourProfile`].
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
    pub fn try_to_curve_region(&self, policy: &CurvePolicy) -> ExactCurveResult<CurveRegion2> {
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
    pub fn new(roles: Vec<CurveRegionLoopRole>, signed_areas: Vec<Real>) -> CurveResult<Self> {
        validate_evidence_length(roles.len(), "signed area", signed_areas.len())?;
        validate_signed_area_roles(&roles, &signed_areas)?;
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
    ) -> CurveResult<Self> {
        validate_evidence_length(roles.len(), "nesting depth", nesting_depths.len())?;
        validate_evidence_length(roles.len(), "signed area", signed_areas.len())?;
        validate_evidence_length(roles.len(), "sample point", sample_points.len())?;
        validate_nesting_depth_roles(&roles, &nesting_depths)?;
        validate_nonzero_signed_area_evidence(&signed_areas)?;
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
    pub fn new(fragments: Vec<BezierSubcurve2>) -> CurveResult<Self> {
        validate_native_boundary_loop(&fragments)?;
        Ok(Self { fragments })
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
    pub fn signed_area(&self) -> CurveResult<Option<Real>> {
        let mut rational_quadratic_cache = RationalQuadraticAreaIntegralCache::default();
        self.signed_area_with_cache(&mut rational_quadratic_cache)
    }

    fn signed_area_with_cache(
        &self,
        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Option<Real>> {
        if self.fragments.is_empty() {
            return Err(CurveError::Topology(
                "Bezier boundary loop signed area requires nonempty fragments".to_owned(),
            ));
        }

        let mut total = Real::zero();
        for fragment in &self.fragments {
            let Some(contribution) =
                fragment.signed_area_contribution_with_cache(rational_quadratic_cache)?
            else {
                return Ok(None);
            };
            total = &total + &contribution;
        }
        Ok(Some(total))
    }

    /// Classifies an exact point against this curved boundary loop.
    ///
    /// The classifier uses exact point incidence followed by a certified
    /// horizontal-ray crossing count. It does not flatten curved fragments.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<ContourPointLocation>> {
        classify_point_against_native_loop(self, point, policy)
    }
}

impl BezierRegion2 {
    /// Constructs a retained region from closed boundary loops.
    pub fn new(boundary_loops: Vec<BezierBoundaryLoop2>) -> CurveResult<Self> {
        validate_bezier_region_loops(&boundary_loops)?;
        Ok(Self { boundary_loops })
    }

    /// Materializes a retained region from a decided arrangement traversal.
    ///
    /// Every traversal chain must be closed and every referenced graph fragment
    /// must be materialized. Open chains and algebraic-boundary fragments are
    /// returned as explicit uncertainty rather than converted to approximate
    /// boundaries.
    pub fn from_arrangement_traversal(
        graph: &BezierArrangementGraph2,
        traversal: &BezierArrangementTraversal2,
    ) -> Classification<Self> {
        let mut loops = Vec::with_capacity(traversal.chains().len());
        for chain in traversal.chains() {
            if !chain.is_closed() {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }

            let mut fragments = Vec::with_capacity(chain.len());
            for index in chain.fragment_indices() {
                let Some(fragment) = graph.fragments().get(*index) else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                match fragment.fragment() {
                    BezierSplitFragment2::Materialized { curve, .. } => {
                        fragments.push(curve.clone());
                    }
                    BezierSplitFragment2::AlgebraicEndpointImages { .. }
                    | BezierSplitFragment2::Unresolved { .. } => {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                }
            }
            let loop_ = match BezierBoundaryLoop2::new(fragments) {
                Ok(loop_) => loop_,
                Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
            };
            loops.push(loop_);
        }

        match Self::new(loops) {
            Ok(region) => Classification::Decided(region),
            Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
        }
    }

    /// Materializes a native region from a resolved linear-overlap traversal.
    ///
    /// This consumes the refined graph carried by
    /// [`BezierRetainedLinearOverlapTraversal2`] instead of asking callers to
    /// manually pair a derived traversal with the derived graph.  It remains a
    /// native-region constructor: if any accepted refined fragment is only an
    /// algebraic endpoint-image carrier, the result is explicit boundary
    /// uncertainty.  The split/refine/traverse evidence stays separate from
    /// region materialization in the exactness model's exact-computation sense.  The positive-dimensional overlap is consumed
    /// only after the degenerate-intersection clipping model degeneracy is recorded
    /// as a resolved span on the refinement evidence.
    pub fn from_retained_linear_overlap_traversal(
        traversal: &BezierRetainedLinearOverlapTraversal2,
    ) -> Classification<Self> {
        Self::from_arrangement_traversal(traversal.refinement().graph(), traversal.traversal())
    }

    /// Materializes a native region from a represented rational-overlap traversal.
    ///
    /// The traversal retains the exact split ranges and refined graph needed to
    /// keep region materialization paired with the geometry it references.
    pub fn from_retained_rational_overlap_traversal(
        traversal: &BezierRetainedRationalOverlapTraversal2,
    ) -> Classification<Self> {
        Self::from_arrangement_traversal(traversal.refinement().graph(), traversal.traversal())
    }

    /// Returns retained native boundary loops.
    pub fn boundary_loops(&self) -> &[BezierBoundaryLoop2] {
        &self.boundary_loops
    }

    /// Consumes the region and returns retained native boundary loops.
    pub fn into_boundary_loops(self) -> Vec<BezierBoundaryLoop2> {
        self.boundary_loops
    }

    /// Returns true when the region has no boundary loops.
    pub fn is_empty(&self) -> bool {
        self.boundary_loops.is_empty()
    }

    /// Returns the number of boundary loops.
    pub fn len(&self) -> usize {
        self.boundary_loops.len()
    }

    /// Returns the exact signed area when all loops have implemented area integrals.
    pub fn signed_area(&self) -> CurveResult<Option<Real>> {
        let mut total = Real::zero();
        for boundary_loop in &self.boundary_loops {
            let Some(area) = boundary_loop.signed_area()? else {
                return Ok(None);
            };
            total = &total + &area;
        }
        Ok(Some(total))
    }
}

fn validate_native_boundary_loop(fragments: &[BezierSubcurve2]) -> CurveResult<()> {
    if fragments.is_empty() {
        return Err(CurveError::Topology(
            "Bezier boundary loop requires nonempty fragments".to_owned(),
        ));
    }

    let policy = CurvePolicy::certified();
    for (left, right) in fragments
        .iter()
        .zip(fragments.iter().cycle().skip(1))
        .take(fragments.len())
    {
        if !certified_points_equal(&left.endpoints().1, &right.endpoints().0, &policy) {
            return Err(CurveError::Topology(
                "Bezier boundary loop fragments must be endpoint-connected and closed".to_owned(),
            ));
        }
    }
    Ok(())
}

fn certified_points_equal(left: &Point2, right: &Point2, policy: &CurvePolicy) -> bool {
    is_zero(&left.distance_squared(right), policy) == Some(true)
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
    pub fn new(fragments: Vec<BezierSplitFragment2>) -> CurveResult<Self> {
        validate_retained_boundary_loop(&fragments)?;
        Ok(Self {
            fragments,
            arrangement_sources: None,
        })
    }

    /// Constructs a retained boundary loop with one source record per fragment.
    pub fn try_new_with_arrangement_sources(
        fragments: Vec<BezierSplitFragment2>,
        arrangement_sources: Vec<CurveRegionFragmentSource2>,
    ) -> CurveResult<Self> {
        validate_retained_boundary_loop(&fragments)?;
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
    ) -> CurveResult<Self> {
        if fragments.is_empty() || fragments.len() != arrangement_sources.len() {
            return Err(CurveError::Topology(
                "certified arrangement chain has inconsistent retained fragments".into(),
            ));
        }
        let policy = CurvePolicy::certified();
        for fragment in &fragments {
            validate_retained_fragment_provenance(fragment, &policy)?;
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

    /// Returns true when any retained fragment has algebraic endpoint images.
    pub fn has_algebraic_fragments(&self) -> bool {
        self.fragments.iter().any(|fragment| {
            matches!(
                fragment,
                BezierSplitFragment2::AlgebraicEndpointImages { .. }
            )
        })
    }

    /// Returns exact signed area only for fully native loops with implemented integrals.
    pub fn signed_area(&self) -> CurveResult<Option<Real>> {
        let mut rational_quadratic_cache = RationalQuadraticAreaIntegralCache::default();
        self.signed_area_with_cache(&mut rational_quadratic_cache)
    }

    fn signed_area_with_cache(
        &self,
        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Option<Real>> {
        if self.fragments.is_empty() {
            return Err(CurveError::Topology(
                "retained Bezier boundary loop signed area requires nonempty fragments".to_owned(),
            ));
        }

        let mut total = Real::zero();
        for fragment in &self.fragments {
            let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
                return Ok(None);
            };
            let Some(contribution) =
                curve.signed_area_contribution_with_cache(rational_quadratic_cache)?
            else {
                return Ok(None);
            };
            total = &total + &contribution;
        }
        Ok(Some(total))
    }
}

fn validate_retained_boundary_loop(fragments: &[BezierSplitFragment2]) -> CurveResult<()> {
    if fragments.is_empty() {
        return Err(CurveError::Topology(
            "retained Bezier boundary loop requires nonempty fragments".to_owned(),
        ));
    }
    for fragment in fragments {
        validate_retained_fragment_provenance(fragment, &CurvePolicy::certified())?;
    }
    validate_retained_boundary_loop_connectivity(fragments, &CurvePolicy::certified())
}

fn validate_retained_fragment_provenance(
    fragment: &BezierSplitFragment2,
    policy: &CurvePolicy,
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
        BezierSplitFragment2::Unresolved { .. } => Err(CurveError::Topology(
            "retained Bezier region boundary loops must not contain unresolved carriers".into(),
        )),
    }
}

fn validate_retained_fragment_parameter_order(
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
            if !image.is_transformed() {
                return Err(CurveError::Topology(
                    "retained algebraic endpoint image must be exact transformed evidence".into(),
                ));
            }
            if let Some(source_curve) = source_curve {
                let expected = crate::BezierAlgebraicEndpointImage2::from_source_curve(
                    source_curve,
                    parameter,
                    policy,
                )?;
                if &expected != image {
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
    algebraic: Option<(
        Box<AlgebraicRootRepresentation>,
        Box<AlgebraicRootRepresentation>,
    )>,
    source: Option<(BezierSubcurve2, BezierParameter2)>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RetainedEndpointEquality {
    Equal,
    NotEqual,
    Uncertified,
}

fn validate_retained_boundary_loop_connectivity(
    fragments: &[BezierSplitFragment2],
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
) -> CurveResult<RetainedEndpointEvidence> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            let (start, end) = curve.endpoints();
            Ok(RetainedEndpointEvidence {
                point: Some(if start_endpoint { start } else { end }),
                algebraic: None,
                source: None,
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
            Ok(RetainedEndpointEvidence {
                point,
                algebraic,
                source,
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
    let (x, y) = match image.point() {
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
    policy: &CurvePolicy,
) -> CurveResult<Option<Point2>> {
    if let Some(image) = image
        && let Some(point) = exact_rational_point_from_image(image.point())
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
    policy: &CurvePolicy,
) -> RetainedEndpointEquality {
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

    if let (Some(left), Some(right)) = (&left.source, &right.source)
        && left == right
    {
        return RetainedEndpointEquality::Equal;
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

struct NativeCurvePathRegion {
    region: LineArcRegion2,
    signed_areas: Vec<Real>,
}

fn native_region_from_curve_paths(
    paths: &[CurvePath2],
    roles: &[CurveRegionLoopRole],
    fill_rules: &[FillRule],
) -> CurveResult<Option<NativeCurvePathRegion>> {
    if paths.len() != roles.len() || paths.len() != fill_rules.len() {
        return Err(CurveError::Topology(
            "native curve-path role and fill-rule counts must match".into(),
        ));
    }

    let mut material = Vec::new();
    let mut holes = Vec::new();
    let mut signed_areas = Vec::with_capacity(paths.len());
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
        let contour = Contour2::try_new_with_fill_rule(segments, *fill_rule)?;
        signed_areas.push(contour.signed_area()?.ok_or_else(|| {
            CurveError::Topology("native line/arc path did not provide an exact signed area".into())
        })?);
        match role {
            CurveRegionLoopRole::Material => material.push(contour),
            CurveRegionLoopRole::Hole => holes.push(contour),
        }
    }
    Ok(Some(NativeCurvePathRegion {
        region: LineArcRegion2::new(material, holes),
        signed_areas,
    }))
}

fn curve_region_promotion_error(cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(CurveOperation2::Construction, CurveFamily2::Line, cause)
}

fn promote_native_region_arrangement(
    arrangement: RegionArrangement2,
    policy: &CurvePolicy,
) -> ExactCurveResult<CurveRegionArrangement2> {
    let (region, workspace, summary) = arrangement.into_region_with_facts();
    let region = region
        .as_ref()
        .map(|region| CurveRegion2::try_from_line_arc_region(region, policy))
        .transpose()?;
    Ok(CurveRegionArrangement2 {
        region,
        workspace,
        summary,
    })
}

fn curve_region_edit_error(operation: CurveOperation2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(operation, CurveFamily2::Line, cause)
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
                        used_exact_native_fast_path: fallback_evidence
                            .used_exact_native_fast_path(),
                        used_certified_parallel_path: false,
                        used_segmented_source_fallback: !fallback_evidence
                            .used_exact_native_fast_path(),
                        max_parallel_fit_error: max_parallel_fit_error.clone(),
                        max_output_chord_error: max_output_chord_error.clone(),
                        certified_pre_regularization_boundary_error: fallback_evidence
                            .used_exact_native_fast_path()
                            .then(Real::zero),
                        final_boundary_hausdorff_certified: fallback_evidence
                            .used_exact_native_fast_path(),
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
    component: LineArcRegion2,
    material_components: &mut Vec<LineArcRegion2>,
    void_components: &mut Vec<LineArcRegion2>,
) {
    match role {
        CurveRegionLoopRole::Material => material_components.push(component),
        CurveRegionLoopRole::Hole => void_components.push(component),
    }
}

fn regularize_native_offset_regions(
    mut material_components: Vec<LineArcRegion2>,
    void_components: Vec<LineArcRegion2>,
    policy: &CurvePolicy,
) -> ExactCurveResult<Classification<LineArcRegion2>> {
    if material_components.len() == 1 && void_components.is_empty() {
        return Ok(Classification::Decided(
            material_components
                .pop()
                .expect("single offset component inventory"),
        ));
    }
    let mut material = LineArcRegion2::empty();
    for component in material_components {
        material = match material
            .boolean_region(&component, BooleanOp::Union, FillRule::NonZero, policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    }

    if material.is_empty() || void_components.is_empty() {
        return Ok(Classification::Decided(material));
    }

    let mut voids = LineArcRegion2::empty();
    for component in void_components {
        voids = match voids
            .boolean_region(&component, BooleanOp::Union, FillRule::NonZero, policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    }
    material
        .boolean_region(&voids, BooleanOp::Difference, FillRule::NonZero, policy)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))
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

const fn curve_region_role_depth(role: CurveRegionLoopRole) -> i32 {
    match role {
        CurveRegionLoopRole::Material => 1,
        CurveRegionLoopRole::Hole => -1,
    }
}

fn replace_native_region_role_contour(
    region: LineArcRegion2,
    role: CurveRegionLoopRole,
    ordinal: usize,
    replacement: Contour2,
) -> CurveResult<LineArcRegion2> {
    let mut material = region.material_contours().to_vec();
    let mut holes = region.hole_contours().to_vec();
    let target = match role {
        CurveRegionLoopRole::Material => material.get_mut(ordinal),
        CurveRegionLoopRole::Hole => holes.get_mut(ordinal),
    }
    .ok_or(CurveError::InvalidCurveRange)?;
    *target = replacement;
    Ok(LineArcRegion2::new(material, holes))
}

impl CurveRegion2 {
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
        policy: &CurvePolicy,
    ) -> ExactCurveResult<CurveRegionArrangement2> {
        let arrangement =
            LineArcRegion2::arrange_unordered_segments(source_segments, fill_rule, policy)
                .map_err(curve_region_promotion_error)?;
        promote_native_region_arrangement(arrangement, policy)
    }

    /// Arranges borrowed unordered exact line/arc segments into unified topology.
    pub fn arrange_unordered_segments_borrowed(
        source_segments: &[Segment2],
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<CurveRegionArrangement2> {
        let arrangement =
            LineArcRegion2::arrange_unordered_segments_borrowed(source_segments, fill_rule, policy)
                .map_err(curve_region_promotion_error)?;
        promote_native_region_arrangement(arrangement, policy)
    }

    /// Arranges unordered exact lines through the specialized line pipeline.
    pub fn arrange_unordered_line_segments(
        source_segments: Vec<LineSeg2>,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<CurveRegionArrangement2> {
        let arrangement =
            LineArcRegion2::arrange_unordered_line_segments(source_segments, fill_rule, policy)
                .map_err(curve_region_promotion_error)?;
        promote_native_region_arrangement(arrangement, policy)
    }

    /// Arranges borrowed unordered exact lines through the specialized line pipeline.
    pub fn arrange_unordered_line_segments_borrowed(
        source_segments: &[LineSeg2],
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<CurveRegionArrangement2> {
        let arrangement = LineArcRegion2::arrange_unordered_line_segments_borrowed(
            source_segments,
            fill_rule,
            policy,
        )
        .map_err(curve_region_promotion_error)?;
        promote_native_region_arrangement(arrangement, policy)
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
        policy: &CurvePolicy,
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
        let mut promoted = Self::try_from_boundary_paths_with_loop_semantics_and_policy(
            &paths,
            &roles,
            &fill_rules,
            policy,
            None,
        )?;
        promoted.line_image_region = Arc::new(OnceLock::new());
        let _ = promoted
            .line_image_region
            .set(Some(LineArcRegion2::new(material_contours, hole_contours)));
        Ok(promoted)
    }

    /// Constructs a unified region whose native contours are all material.
    pub fn try_from_native_material_contours(
        material_contours: Vec<Contour2>,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Self> {
        Self::try_from_native_contours(material_contours, Vec::new(), policy)
    }

    /// Nests unordered native boundary contours and promotes their decided roles.
    ///
    /// Even containment depth becomes material and odd depth becomes a hole,
    /// matching `LineArcRegion2::from_boundary_contours`. Intersecting, touching, or
    /// otherwise uncertifiable boundaries remain an explicit uncertainty.
    pub fn try_from_native_boundary_contours(
        contours: Vec<Contour2>,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<Self>> {
        Self::try_from_native_boundary_contours_with_evidence(contours, policy)
            .map(CurveRegionBoundaryContourBuildResult2::into_region_classification)
    }

    /// Borrowed counterpart to [`Self::try_from_native_boundary_contours`].
    pub fn try_from_native_boundary_contours_borrowed(
        contours: &[Contour2],
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<Self>> {
        Self::try_from_native_boundary_contours(contours.to_vec(), policy)
    }

    /// Nests native contours with evidence evidence and returns unified topology.
    ///
    /// The specialized line/arc engine performs intersection validation,
    /// containment-depth assignment, and material/hole binning. Any decided
    /// output is immediately promoted to `CurveRegion2`; callers never need to
    /// own or inspect the transient [`LineArcRegion2`] result.
    pub(crate) fn try_from_native_boundary_contours_with_evidence(
        contours: Vec<Contour2>,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<CurveRegionBoundaryContourBuildResult2> {
        let built = LineArcRegion2::from_boundary_contours_with_evidence(contours, policy)
            .map_err(curve_region_promotion_error)?;
        let (region, evidence) = built.into_parts();
        let region = region
            .as_ref()
            .map(|region| Self::try_from_line_arc_region(region, policy))
            .transpose()?;
        Ok(CurveRegionBoundaryContourBuildResult2 { region, evidence })
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
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<Self>> {
        match contour
            .regularize_self_intersections_native(policy)
            .map_err(curve_region_promotion_error)?
        {
            Classification::Decided(region) => {
                Self::try_from_line_arc_region(&region, policy).map(Classification::Decided)
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Losslessly promotes a native line/arc region into the mixed-family carrier.
    ///
    /// Material and hole roles come from the source region rather than being
    /// inferred again from loop nesting or authored orientation. The original
    /// native region is retained as the certified line/arc fast path, preserving
    /// its contour fill rules and query behavior in the canonical mixed-family
    /// API.
    #[doc(hidden)]
    pub fn try_from_line_arc_region(
        region: &LineArcRegion2,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Self> {
        Self::try_from_native_contours(
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
    ) -> ExactCurveResult<Self> {
        Self::try_from_boundary_paths_with_loop_semantics_and_policy(
            paths,
            roles,
            fill_rules,
            &CurvePolicy::certified(),
            None,
        )
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
    ) -> ExactCurveResult<Self> {
        let mut region =
            Self::try_from_boundary_paths_with_loop_semantics(paths, roles, fill_rules)?;
        region.signed_loop_composition = true;
        Ok(region)
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
    ) -> ExactCurveResult<Self> {
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
        Self::try_from_boundary_paths_with_loop_semantics_and_policy(
            paths,
            roles,
            fill_rules,
            &CurvePolicy::certified(),
            Some(
                interior_sides
                    .iter()
                    .map(|side| *side == CurveBoundaryInteriorSide2::Left)
                    .collect(),
            ),
        )
    }

    fn try_from_boundary_paths_with_loop_semantics_and_policy(
        paths: &[CurvePath2],
        roles: &[CurveRegionLoopRole],
        fill_rules: &[FillRule],
        policy: &CurvePolicy,
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
        let mut region = Self::try_from_boundary_paths(paths)?;
        region.certified_loop_roles = Some(Arc::from(roles));
        region.certified_loop_fill_rules = Some(Arc::from(fill_rules));
        if let Some(filled_sides) = certified_filled_sides {
            region = region
                .with_certified_filled_side_is_left(filled_sides)
                .map_err(curve_region_promotion_error)?;
        }
        if let Some(native) = native_region_from_curve_paths(paths, roles, fill_rules)
            .map_err(curve_region_promotion_error)?
        {
            if region.filled_side_is_left.get().is_none()
                && let Ok(filled_sides) =
                    filled_sides_from_roles_and_areas(roles, &native.signed_areas, policy)
            {
                region = region
                    .with_certified_filled_side_is_left(filled_sides)
                    .map_err(curve_region_promotion_error)?;
            }
            let _ = region.line_image_region.set(Some(native.region));
        }
        Ok(region)
    }

    /// Constructs a top-level exact curved region from closed boundary paths.
    ///
    /// Every authored family is promoted through its clone-shared native
    /// topology once.
    pub fn try_from_boundary_paths(paths: &[CurvePath2]) -> ExactCurveResult<Self> {
        let mut boundary_loops = Vec::with_capacity(paths.len());
        let mut next_arrangement_fragment_index = 0;
        for path in paths {
            path.bezier_boundary_loop()
                .map_err(|error| error.with_operation(CurveOperation2::Construction))?;
            let fragment_capacity = path.native_bezier_fragments()?.len();
            let mut fragments = Vec::with_capacity(fragment_capacity);
            let mut arrangement_sources = Vec::with_capacity(fragment_capacity);
            for curve in path.curves() {
                for native in curve.native_bezier_fragments()? {
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
        policy: &CurvePolicy,
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

        let mut loops = Vec::with_capacity(self.boundary_loops.len());
        for boundary in &self.boundary_loops {
            let fragments = boundary
                .fragments()
                .iter()
                .map(|fragment| {
                    transform_retained_region_fragment(fragment, m00, m01, m10, m11, tx, ty, policy)
                })
                .collect::<ExactCurveResult<Vec<_>>>()?;
            let boundary = match boundary.arrangement_sources() {
                Some(sources) => {
                    CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                        fragments,
                        sources.to_vec(),
                    )
                }
                None => CurveRegionBoundaryLoop2::new(fragments),
            }
            .map_err(affine_region_error)?;
            loops.push(boundary);
        }
        let mut transformed = Self::new(loops).map_err(affine_region_error)?;
        transformed.certified_loop_roles = self.certified_loop_roles.clone();
        transformed.certified_loop_fill_rules = self.certified_loop_fill_rules.clone();
        transformed.signed_loop_composition = self.signed_loop_composition;
        let sides = match self
            .filled_side_is_left(policy)
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
            let _ = transformed.line_image_region.set(Some(region));
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

    fn from_certified_boundary_loops(boundary_loops: Vec<CurveRegionBoundaryLoop2>) -> Self {
        Self {
            boundary_loops,
            certified_loop_roles: None,
            certified_loop_fill_rules: None,
            signed_loop_composition: false,
            filled_side_is_left: Arc::new(OnceLock::new()),
            native_boundary_loops: Arc::new(OnceLock::new()),
            native_boundary_bounds: Arc::new(OnceLock::new()),
            line_image_region: Arc::new(OnceLock::new()),
            retained_rational_evaluators: Arc::new(OnceLock::new()),
            signed_area_cache: Arc::new(OnceLock::new()),
        }
    }

    pub(crate) fn with_certified_filled_side_is_left(
        self,
        filled_side_is_left: Vec<bool>,
    ) -> CurveResult<Self> {
        if filled_side_is_left.len() != self.boundary_loops.len() {
            return Err(CurveError::Topology(
                "curved-region filled-side evidence must match the boundary-loop count".into(),
            ));
        }
        let _ = self
            .filled_side_is_left
            .set(Ok(Classification::Decided(Arc::from(filled_side_is_left))));
        Ok(self)
    }

    pub fn filled_side_is_left(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<&[bool]>> {
        let mut rational_quadratic_cache = RationalQuadraticAreaIntegralCache::default();
        self.filled_side_is_left_with_area_cache(policy, &mut rational_quadratic_cache)
    }

    pub(crate) fn filled_side_is_left_with_area_cache(
        &self,
        policy: &CurvePolicy,
        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Classification<&[bool]>> {
        match self.filled_side_is_left.get_or_init(|| {
            self.compute_filled_side_is_left_with_area_cache(policy, rational_quadratic_cache)
        }) {
            Ok(Classification::Decided(sides)) => Ok(Classification::Decided(sides.as_ref())),
            Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(*reason)),
            Err(error) => Err(error.clone()),
        }
    }

    fn compute_filled_side_is_left_with_area_cache(
        &self,
        policy: &CurvePolicy,
        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Classification<Arc<[bool]>>> {
        if let Some(roles) = self.certified_loop_roles.as_deref() {
            let signed_areas = self
                .boundary_loops
                .iter()
                .map(|boundary_loop| boundary_loop.signed_area_with_cache(rational_quadratic_cache))
                .collect::<CurveResult<Vec<_>>>()?
                .into_iter()
                .collect::<Option<Vec<_>>>();
            if let Some(signed_areas) = signed_areas {
                return filled_sides_from_roles_and_areas(roles, &signed_areas, policy)
                    .map(|sides| Classification::Decided(Arc::from(sides)));
            }
        }
        if self.boundary_loops.len() == 1
            && let Some(area) =
                self.boundary_loops[0].signed_area_with_cache(rational_quadratic_cache)?
        {
            return Ok(match real_sign(&area, policy) {
                Some(RealSign::Positive) => Classification::Decided(Arc::from([true].as_slice())),
                Some(RealSign::Negative) => Classification::Decided(Arc::from([false].as_slice())),
                Some(RealSign::Zero) => Classification::Uncertain(UncertaintyReason::Boundary),
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            });
        }

        match self.curved_nesting_role_evidence(policy)? {
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

        match self.line_image_role_evidence(policy)? {
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
    /// [`BezierRegion2::from_arrangement_traversal`] but preserves algebraic
    /// carriers instead of requiring native subcurves.
    pub fn from_retained_arrangement_traversal(
        graph: &BezierArrangementGraph2,
        traversal: &BezierArrangementTraversal2,
    ) -> Classification<Self> {
        Self::from_retained_arrangement_traversal_impl(graph, traversal, true)
    }

    pub(crate) fn from_certified_retained_arrangement_traversal(
        graph: &BezierArrangementGraph2,
        traversal: &BezierArrangementTraversal2,
    ) -> Classification<Self> {
        Self::from_retained_arrangement_traversal_impl(graph, traversal, false)
    }

    fn from_retained_arrangement_traversal_impl(
        graph: &BezierArrangementGraph2,
        traversal: &BezierArrangementTraversal2,
        validate_provenance: bool,
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
                    | BezierSplitFragment2::AlgebraicEndpointImages { .. } => {
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
            if validate_provenance
                && validate_retained_arrangement_chain_connectivity(
                    graph,
                    chain.fragment_indices(),
                    &CurvePolicy::certified(),
                )
                .is_err()
            {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            let loop_ = if validate_provenance {
                match CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                    fragments,
                    arrangement_sources,
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

        if validate_provenance {
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
    ) -> Classification<Self> {
        Self::from_retained_arrangement_traversal(
            traversal.refinement().graph(),
            traversal.traversal(),
        )
    }

    /// Materializes retained carriers from a represented rational-overlap traversal.
    ///
    /// Native and algebraic endpoint-image fragments remain exact retained
    /// objects; unresolved carriers and open chains remain explicit uncertainty.
    pub fn from_retained_rational_overlap_traversal(
        traversal: &BezierRetainedRationalOverlapTraversal2,
    ) -> Classification<Self> {
        Self::from_retained_arrangement_traversal(
            traversal.refinement().graph(),
            traversal.traversal(),
        )
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
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveRegionLineRoleEvidence2>> {
        let mut contours = Vec::with_capacity(self.boundary_loops.len());
        let mut materialized_fragment_count = 0_usize;
        let mut algebraic_fragment_count = 0_usize;
        for boundary_loop in &self.boundary_loops {
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
        .with_loop_arrangement_sources(retained_loop_arrangement_sources(&self.boundary_loops))?;
        Ok(Classification::Decided(evidence))
    }

    /// Assigns material/hole roles from exact native loop signed-area orientation.
    ///
    /// A negative signed area is treated as a material loop and a positive
    /// signed area as a hole loop, matching the current Bezier region boundary
    /// convention used by [`BezierRegion2::signed_area`].  This method is a
    /// evidence-bearing orientation adapter: it does not infer nesting and it
    /// does not sample nonlinear loops.  Use [`Self::line_image_role_evidence`]
    /// when exact line-image nesting is required.
    pub fn signed_area_role_evidence(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveRegionSignedAreaRoleEvidence2>> {
        let mut roles = Vec::with_capacity(self.boundary_loops.len());
        let mut signed_areas = Vec::with_capacity(self.boundary_loops.len());
        for boundary_loop in &self.boundary_loops {
            let Some(area) = boundary_loop.signed_area()? else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
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
        let evidence = CurveRegionSignedAreaRoleEvidence2::new(roles, signed_areas)?
            .with_loop_fragment_counts(retained_loop_fragment_counts(&self.boundary_loops))?
            .with_loop_arrangement_sources(retained_loop_arrangement_sources(
                &self.boundary_loops,
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
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveRegionNestingRoleEvidence2>> {
        let Some(native_loops) = self.native_boundary_loops() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let native_bounds = self.native_boundary_bounds(policy);
        let mut sample_points = Vec::with_capacity(self.boundary_loops.len());
        let mut signed_areas = Vec::with_capacity(self.boundary_loops.len());
        for native_loop in native_loops {
            let Some(area) = native_loop.signed_area()? else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
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
        )?
        .with_loop_fragment_counts(retained_loop_fragment_counts(&self.boundary_loops))?
        .with_loop_arrangement_sources(retained_loop_arrangement_sources(&self.boundary_loops))?;
        Ok(Classification::Decided(evidence))
    }

    /// Returns one exact material/hole role per retained loop.
    ///
    /// The strongest curved nesting classifier is preferred. Exact signed-area
    /// orientation and line-image nesting are retained fallbacks for carrier
    /// subsets that do not support the full curved evidence.
    pub fn loop_roles(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<CurveRegionLoopRole>>> {
        if let Some(roles) = &self.certified_loop_roles {
            return Ok(Classification::Decided(roles.to_vec()));
        }
        match self.curved_nesting_role_evidence(policy)? {
            Classification::Decided(evidence) => {
                return Ok(Classification::Decided(evidence.roles().to_vec()));
            }
            Classification::Uncertain(_) => {}
        }
        match self.signed_area_role_evidence(policy)? {
            Classification::Decided(evidence) => {
                return Ok(Classification::Decided(evidence.roles().to_vec()));
            }
            Classification::Uncertain(_) => {}
        }
        self.line_image_role_evidence(policy)
            .map(|roles| roles.map(|evidence| evidence.roles().to_vec()))
    }

    /// Returns the number of material and hole loops in authoritative topology.
    ///
    /// The tuple is `(material, holes)`. Role classification follows the same
    /// exact retained-curve path as [`CurveRegion2::loop_roles`]; no native
    /// [`LineArcRegion2`] projection is required.
    pub fn loop_role_counts(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<(usize, usize)>> {
        self.loop_roles(policy).map(|roles| {
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
        self.certified_loop_fill_rules.as_deref()
    }

    /// Groups retained material loops with their exact owned hole loops.
    ///
    /// Roles come from [`Self::loop_roles`]. Each hole contributes a retained
    /// exact endpoint witness which is classified against material carriers;
    /// sampled coordinates and winding are not used. An algebraic endpoint
    /// without a represented point remains explicit uncertainty.
    pub fn boundary_profiles(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<CurveRegionProfile2<'_>>>> {
        let roles = match self.loop_roles(policy)? {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if roles.len() != self.boundary_loops.len() {
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
                    material: &self.boundary_loops[index],
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
            let point = match retained_loop_sample_point(&self.boundary_loops[hole_index], policy)?
            {
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
            profiles[owner].holes.push(&self.boundary_loops[hole_index]);
        }
        Ok(Classification::Decided(profiles))
    }

    /// Returns the certified internal line/arc accelerator when this region has one.
    pub(crate) fn native_line_arc_region(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<&LineArcRegion2>> {
        if self.line_image_region.get().is_none() {
            if self.certified_loop_roles.is_some() {
                match self.certified_line_image_region(policy)? {
                    Classification::Decided(region) => {
                        let _ = self.line_image_region.set(Some(region));
                    }
                    Classification::Uncertain(UncertaintyReason::Unsupported) => {
                        let _ = self.line_image_region.set(None);
                    }
                    Classification::Uncertain(_) => {}
                }
            } else {
                match self.line_image_role_evidence(policy)? {
                    Classification::Decided(evidence) => {
                        let region = self.region_from_line_role_evidence(&evidence)?;
                        let _ = self.line_image_region.set(Some(region));
                    }
                    Classification::Uncertain(UncertaintyReason::Unsupported) => {
                        let _ = self.line_image_region.set(None);
                    }
                    Classification::Uncertain(_) => {}
                }
            }
        }
        Ok(
            match self.line_image_region.get().and_then(Option::as_ref) {
                Some(region) => Classification::Decided(region),
                None => Classification::Uncertain(UncertaintyReason::Unsupported),
            },
        )
    }

    /// Returns borrowed native line/arc contours without exposing `LineArcRegion2` ownership.
    ///
    /// This is the public acceleration boundary for consumers that genuinely
    /// need native primitives. Higher-order topology
    /// remains explicit `Unsupported` uncertainty and is never segmented.
    pub fn native_contours_fast_path(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveRegionNativeContourView2<'_>>> {
        self.native_line_arc_region(policy).map(|native| {
            native.map(|native| CurveRegionNativeContourView2 {
                material_contours: native.material_contours(),
                hole_contours: native.hole_contours(),
            })
        })
    }

    /// Chamfers one boundary-loop vertex without leaving the unified carrier.
    ///
    /// `loop_index` addresses this region's retained boundary order. The edited
    /// Native line/arc topology uses its certified contour fast path. Fully
    /// materialized higher-order loops use exact `CurvePath2` subdivision and
    /// are rebuilt with their material/hole and fill semantics intact.
    /// Algebraic-endpoint fragments that cannot be materialized remain explicit
    /// `Unsupported` uncertainty.
    pub fn chamfer_loop_vertex_by_parameters(
        &self,
        loop_index: usize,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<Self>> {
        let (region, role, ordinal) =
            match self.native_region_loop_for_edit(loop_index, CurveOperation2::Chamfer, policy)? {
                Classification::Decided(edit) => edit,
                Classification::Uncertain(UncertaintyReason::Unsupported) => {
                    let mut paths = match self
                        .materialized_boundary_paths_for_edit(CurveOperation2::Chamfer)?
                    {
                        Classification::Decided(paths) => paths,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    paths[loop_index] = paths[loop_index].chamfer_vertex_by_parameters(
                        vertex_index,
                        previous_param,
                        next_param,
                    )?;
                    return self.rebuild_after_materialized_path_edit(
                        paths,
                        CurveOperation2::Chamfer,
                        policy,
                    );
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let contour = native_region_role_contour(&region, role, ordinal).ok_or_else(|| {
            curve_region_edit_error(CurveOperation2::Chamfer, CurveError::InvalidCurveRange)
        })?;
        let chamfer = contour
            .chamfer_vertex_by_parameters(vertex_index, previous_param, next_param, policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Chamfer, cause))?;
        let contour = match chamfer {
            Classification::Decided(contour) => contour,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let edited = replace_native_region_role_contour(region, role, ordinal, contour)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Chamfer, cause))?;
        Self::try_from_line_arc_region(&edited, policy).map(Classification::Decided)
    }

    /// Fillets one boundary-loop vertex without leaving the unified carrier.
    ///
    /// The exact trim parameters, center, and sweep direction are validated by
    /// either the native contour fast path or higher-order `CurvePath2` editing.
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
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<Self>> {
        let (region, role, ordinal) =
            match self.native_region_loop_for_edit(loop_index, CurveOperation2::Fillet, policy)? {
                Classification::Decided(edit) => edit,
                Classification::Uncertain(UncertaintyReason::Unsupported) => {
                    let mut paths =
                        match self.materialized_boundary_paths_for_edit(CurveOperation2::Fillet)? {
                            Classification::Decided(paths) => paths,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    paths[loop_index] = paths[loop_index].fillet_vertex_by_parameters(
                        vertex_index,
                        previous_param,
                        next_param,
                        center,
                        clockwise,
                    )?;
                    return self.rebuild_after_materialized_path_edit(
                        paths,
                        CurveOperation2::Fillet,
                        policy,
                    );
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let contour = native_region_role_contour(&region, role, ordinal).ok_or_else(|| {
            curve_region_edit_error(CurveOperation2::Fillet, CurveError::InvalidCurveRange)
        })?;
        let fillet = contour
            .fillet_vertex_by_parameters(
                vertex_index,
                previous_param,
                next_param,
                center,
                clockwise,
                policy,
            )
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?;
        let contour = match fillet {
            Classification::Decided(contour) => contour,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let edited = replace_native_region_role_contour(region, role, ordinal, contour)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Fillet, cause))?;
        Self::try_from_line_arc_region(&edited, policy).map(Classification::Decided)
    }

    fn materialized_boundary_paths_for_edit(
        &self,
        operation: CurveOperation2,
    ) -> ExactCurveResult<Classification<Vec<CurvePath2>>> {
        let mut paths = Vec::with_capacity(self.boundary_loops.len());
        for boundary_loop in &self.boundary_loops {
            let mut curves = Vec::with_capacity(boundary_loop.fragments().len());
            for fragment in boundary_loop.fragments() {
                let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                curves.push(Curve2::from(curve.clone()));
            }
            paths.push(
                CurvePath2::try_new(curves).map_err(|error| error.with_operation(operation))?,
            );
        }
        Ok(Classification::Decided(paths))
    }

    /// Materializes every retained boundary loop as an exact top-level path.
    ///
    /// Native and represented polynomial/rational fragments preserve their
    /// exact curve carriers and source parameters. Regions whose traversal
    /// still contains an algebraic endpoint that cannot be represented by a
    /// public [`Curve2`] return explicit `Unsupported` uncertainty rather than
    /// segmenting the boundary. This is the lossless interchange counterpart
    /// to [`CurveRegion2::segment_to_finite_profiles`](crate::CurveRegion2::segment_to_finite_profiles).
    pub fn materialized_boundary_paths(&self) -> ExactCurveResult<Classification<Vec<CurvePath2>>> {
        self.materialized_boundary_paths_for_edit(CurveOperation2::NativeTopology)
    }

    fn rebuild_after_materialized_path_edit(
        &self,
        paths: Vec<CurvePath2>,
        operation: CurveOperation2,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<Self>> {
        let roles = match self
            .loop_roles(policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let fill_rules = self
            .certified_loop_fill_rules
            .as_deref()
            .map_or_else(|| vec![FillRule::EvenOdd; paths.len()], <[_]>::to_vec);
        Self::try_from_boundary_paths_with_loop_semantics_and_policy(
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
    /// Use [`Self::segment_to_finite_profiles`] for direct mesh/IO output and
    /// [`Self::recover_from_finite_profiles`] for its reconstruction counterpart.
    pub fn segment_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<CurveRegionCertifiedSegmentationResult2>> {
        let paths = match self.materialized_boundary_paths()? {
            Classification::Decided(paths) => paths,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let roles = match self
            .loop_roles(policy)
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

        let region = Self::try_from_native_contours(material, holes, policy)?;
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
    /// The current materialization path handles certified native line/arc
    /// topology. Expanding raw self-walks are regularized exactly. Certified
    /// convex all-line contractions use exact shifted-half-plane intersection
    /// and decide complete collapse. Axis-aligned non-convex line contours use
    /// an exact rectangular distance arrangement that handles neck collapse and
    /// component splitting. Other non-convex contracting wavefronts retain
    /// source-direction evidence where it certifies a branch and otherwise
    /// remain explicit `Unsupported` uncertainty. General polynomial/rational
    /// offsets are likewise unsupported because their exact parallels are not
    /// usually finite rational curves.
    pub fn offset(
        &self,
        distance: Real,
        policy: &CurvePolicy,
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
            .loop_roles(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let filled_sides = match self
            .filled_side_is_left(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(sides) => sides,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if roles.len() != self.boundary_loops.len()
            || filled_sides.len() != self.boundary_loops.len()
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
            let raw_offset = match contour
                .offset_left_with_line_joins(signed_left_distance.clone(), policy)
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
                                    contour.map_or_else(LineArcRegion2::empty, |contour| {
                                        LineArcRegion2::from_material_contours(vec![contour])
                                    }),
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
                match contour
                    .offset_left_orthogonal_line_erosion(signed_left_distance.clone(), policy)
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
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
                match raw_offset
                    .regularize_contracting_line_offset_native(policy)
                    .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
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
                            contour.map_or_else(LineArcRegion2::empty, |contour| {
                                LineArcRegion2::from_material_contours(vec![contour])
                            }),
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
                    LineArcRegion2::from_material_contours(vec![raw_offset])
                }
                Classification::Decided(true) if component_expands => {
                    match raw_offset
                        .regularize_self_intersections_native(policy)
                        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
                    {
                        Classification::Decided(region) => region,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
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
            match regularize_native_offset_regions(material_components, void_components, policy)? {
                Classification::Decided(region) => region,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        Self::try_from_line_arc_region(&edited, policy).map(Classification::Decided)
    }

    /// Offsets arbitrary materialized curve families through certified exact-scalar segmentation.
    ///
    /// Native line/arc topology first uses [`Self::offset`] with no loss. When
    /// that exact fast path evidence `Unsupported`, each retained Bezier or
    /// rational span is subdivided until its control hull certifies the
    /// requested source-curve chord error. The emitted vertices remain
    /// [`Real`] values, and the resulting line topology is offset and
    /// regularized by the exact native kernel. The evidence explicitly marks
    /// this as a lossy boundary: the certificate bounds source-to-chord error,
    /// not Hausdorff error of the final parallel curve.
    pub fn offset_with_certified_segmentation(
        &self,
        distance: Real,
        options: &BezierFlatteningOptions,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<CurveRegionSegmentedOffsetResult2>> {
        match self.offset(distance.clone(), policy)? {
            Classification::Decided(region) => {
                return Ok(Classification::Decided(CurveRegionSegmentedOffsetResult2 {
                    region,
                    evidence: CurveRegionSegmentedOffsetEvidence2 {
                        used_exact_native_fast_path: true,
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

        let segmented = match self.segment_certified(options, policy)? {
            Classification::Decided(segmented) => segmented,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (segmented_source, segmentation_evidence) = segmented.into_parts();
        let region = match segmented_source.offset(distance, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(CurveRegionSegmentedOffsetResult2 {
            region,
            evidence: CurveRegionSegmentedOffsetEvidence2 {
                used_exact_native_fast_path: false,
                max_source_chord_error: segmentation_evidence.max_source_chord_error,
                loop_evidence: segmentation_evidence.loop_evidence,
                lossy_boundary: segmentation_evidence.lossy_boundary,
            },
        }))
    }

    /// Offsets general smooth polynomial boundaries through certified analytic parallels.
    ///
    /// The method first preserves the exact native line/arc result. Otherwise
    /// it constructs exact PH or conservatively verified Blend2D parallels for
    /// every smooth boundary path, chordizes those *output* curves with a
    /// separate certificate, and regularizes the resulting line arrangement.
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
        parallel_options: &BezierParallelVerificationOptions,
        output_flattening: &BezierFlatteningOptions,
        fallback_source_flattening: &BezierFlatteningOptions,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<CurveRegionCertifiedParallelOffsetResult2>> {
        match self.offset(distance.clone(), policy)? {
            Classification::Decided(region) => {
                return Ok(Classification::Decided(
                    CurveRegionCertifiedParallelOffsetResult2 {
                        region,
                        evidence: CurveRegionCertifiedParallelOffsetEvidence2 {
                            used_exact_native_fast_path: true,
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

        let paths = match self.materialized_boundary_paths()? {
            Classification::Decided(paths) => paths,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let roles = match self
            .loop_roles(policy)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let filled_sides = match self
            .filled_side_is_left(policy)
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
                self.offset_with_certified_segmentation(
                    distance,
                    fallback_source_flattening,
                    policy,
                )?,
                parallel_options.max_error(),
                output_flattening.max_error(),
            );
        }

        let parallel_region = Self::try_from_boundary_paths_with_loop_semantics_and_policy(
            &parallel_paths,
            roles.as_ref(),
            &fill_rules,
            policy,
            Some(filled_sides.as_ref().to_vec()),
        )?;
        let segmented = match parallel_region.segment_certified(output_flattening, policy)? {
            Classification::Decided(segmented) => segmented,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let segmented_region = segmented.region();
        let native = segmented_region
            .line_image_region
            .get()
            .and_then(Option::as_ref)
            .expect("certified segmentation always installs native line topology");
        let mut material_components = Vec::new();
        let mut void_components = Vec::new();
        for contour in native.material_contours() {
            let component = match contour
                .regularize_self_intersections_native(policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(component) => component,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            material_components.push(component);
        }
        for contour in native.hole_contours() {
            let component = match contour
                .regularize_self_intersections_native(policy)
                .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
            {
                Classification::Decided(component) => component,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            void_components.push(component);
        }
        let regularized =
            match regularize_native_offset_regions(material_components, void_components, policy)? {
                Classification::Decided(regularized) => regularized,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let region = Self::try_from_line_arc_region(&regularized, policy)?;
        let certified_pre_regularization_boundary_error =
            parallel_options.max_error() + output_flattening.max_error();
        Ok(Classification::Decided(
            CurveRegionCertifiedParallelOffsetResult2 {
                region,
                evidence: CurveRegionCertifiedParallelOffsetEvidence2 {
                    used_exact_native_fast_path: false,
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

    fn native_region_loop_for_edit(
        &self,
        loop_index: usize,
        operation: CurveOperation2,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<(LineArcRegion2, CurveRegionLoopRole, usize)>> {
        if loop_index >= self.boundary_loops.len() {
            return Err(curve_region_edit_error(
                operation,
                CurveError::InvalidCurveRange,
            ));
        }
        let region = match self
            .native_line_arc_region(policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let roles = match self
            .loop_roles(policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let role = roles[loop_index];
        let ordinal = roles[..loop_index]
            .iter()
            .filter(|candidate| **candidate == role)
            .count();
        Ok(Classification::Decided((region.clone(), role, ordinal)))
    }

    fn region_from_line_role_evidence(
        &self,
        evidence: &CurveRegionLineRoleEvidence2,
    ) -> CurveResult<LineArcRegion2> {
        let roles = self
            .certified_loop_roles
            .as_deref()
            .unwrap_or_else(|| evidence.roles());
        self.region_from_line_contours(evidence.contours(), roles)
    }

    fn certified_line_image_region(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<LineArcRegion2>> {
        let Some(roles) = self.certified_loop_roles.as_deref() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let mut contours = Vec::with_capacity(self.boundary_loops.len());
        for boundary_loop in &self.boundary_loops {
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
            let contour = match &self.certified_loop_fill_rules {
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
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RegionPointLocation>> {
        if !self.signed_loop_composition
            && let Some(Some(region)) = self.line_image_region.get()
        {
            return Ok(region.classify_point(point, policy));
        }
        if self.line_image_region.get().is_none() && self.certified_loop_roles.is_some() {
            match self.certified_line_image_region(policy)? {
                Classification::Decided(region) => {
                    let _ = self.line_image_region.set(Some(region));
                }
                Classification::Uncertain(UncertaintyReason::Unsupported) => {
                    let _ = self.line_image_region.set(None);
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let Some(native_loops) = self.native_boundary_loops() else {
            if let Some(region) = self.line_image_region.get() {
                return match region {
                    Some(region) => Ok(region.classify_point(point, policy)),
                    None => classify_point_against_retained_loops(
                        &self.boundary_loops,
                        self.retained_rational_evaluators()?,
                        point,
                        policy,
                        self.certified_loop_roles.as_deref(),
                        self.certified_loop_fill_rules.as_deref(),
                    ),
                };
            }
            return match self.line_image_role_evidence(policy)? {
                Classification::Decided(evidence) => {
                    let region = self.region_from_line_role_evidence(&evidence)?;
                    let _ = self.line_image_region.set(Some(region));
                    Ok(self
                        .line_image_region
                        .get()
                        .expect("decided line-image region was retained")
                        .as_ref()
                        .expect("decided line-image cache contains a region")
                        .classify_point(point, policy))
                }
                Classification::Uncertain(UncertaintyReason::Unsupported) => {
                    let _ = self.line_image_region.set(None);
                    classify_point_against_retained_loops(
                        &self.boundary_loops,
                        self.retained_rational_evaluators()?,
                        point,
                        policy,
                        self.certified_loop_roles.as_deref(),
                        self.certified_loop_fill_rules.as_deref(),
                    )
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        };
        if self
            .certified_loop_roles
            .as_ref()
            .is_some_and(|roles| roles.len() != native_loops.len())
            || self
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
                    if let Some(roles) = &self.certified_loop_roles {
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
    /// integer, matching [`LineArcRegion2::signed_depth`].
    pub fn signed_depth(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<i32>> {
        if !self.signed_loop_composition {
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
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<i32>> {
        let roles = match self.loop_roles(policy)? {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if roles.len() != self.boundary_loops.len() {
            return Err(CurveError::Topology(
                "curve-region signed-depth roles are inconsistent with boundary loops".into(),
            ));
        }
        if self
            .certified_loop_fill_rules
            .as_ref()
            .is_some_and(|rules| rules.len() != self.boundary_loops.len())
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
        if evaluators.len() != self.boundary_loops.len() {
            return Err(CurveError::Topology(
                "curve-region signed-depth evaluator cache is inconsistent with boundary loops"
                    .into(),
            ));
        }
        for (index, ((boundary_loop, evaluators), role)) in self
            .boundary_loops
            .iter()
            .zip(evaluators)
            .zip(&roles)
            .enumerate()
        {
            let fill_rule = self
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
        &self.boundary_loops
    }

    /// Consumes the region and returns retained boundary loops.
    pub fn into_boundary_loops(self) -> Vec<CurveRegionBoundaryLoop2> {
        self.boundary_loops
    }

    /// Returns true when the region has no boundary loops.
    pub fn is_empty(&self) -> bool {
        self.boundary_loops.is_empty()
    }

    /// Returns the number of retained boundary loops.
    pub fn len(&self) -> usize {
        self.boundary_loops.len()
    }

    /// Returns true when any boundary loop retains algebraic endpoint images.
    pub fn has_algebraic_fragments(&self) -> bool {
        self.boundary_loops
            .iter()
            .any(CurveRegionBoundaryLoop2::has_algebraic_fragments)
    }

    /// Returns exact signed area only when all retained loops are native
    /// polynomial loops with implemented Green integrals.
    pub fn signed_area(&self) -> CurveResult<Option<Real>> {
        self.signed_area_cache
            .get_or_init(|| self.compute_signed_area())
            .clone()
    }

    /// Returns exact material-minus-hole area when every loop has an implemented integral.
    ///
    /// Unlike [`Self::signed_area`], this query uses explicit/nesting-derived
    /// loop roles and ignores authored orientation. Nested material islands add
    /// area while owned holes subtract it, matching [`LineArcRegion2::filled_area`].
    /// Per-loop fill rules are applied to repeated windings before role
    /// accumulation. A retained algebraic or otherwise unsupported integral
    /// returns `Decided(None)` rather than approximating the boundary. If exact
    /// self-contact analysis cannot certify a non-repeated loop as simple, the
    /// query remains explicitly uncertain instead of treating traversal
    /// multiplicity as filled-set area.
    pub fn filled_area(&self, policy: &CurvePolicy) -> CurveResult<Classification<Option<Real>>> {
        let mut magnitudes = Vec::with_capacity(self.boundary_loops.len());
        if self
            .certified_loop_fill_rules
            .as_ref()
            .is_some_and(|rules| rules.len() != self.boundary_loops.len())
        {
            return Err(CurveError::Topology(
                "curve-region filled-area fill rules are inconsistent with boundary loops".into(),
            ));
        }
        for (index, boundary_loop) in self.boundary_loops.iter().enumerate() {
            let Some(area) = boundary_loop.signed_area()? else {
                return Ok(Classification::Decided(None));
            };
            let fill_rule = self
                .certified_loop_fill_rules
                .as_ref()
                .map_or(FillRule::EvenOdd, |rules| rules[index]);
            let magnitude = match curve_region_loop_filled_area_magnitude(
                boundary_loop,
                area,
                fill_rule,
                policy,
            )? {
                Classification::Decided(Some(magnitude)) => magnitude,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            magnitudes.push(magnitude);
        }
        let roles = match self.loop_roles(policy)? {
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

    fn compute_signed_area(&self) -> CurveResult<Option<Real>> {
        let mut total = Real::zero();
        for boundary_loop in &self.boundary_loops {
            let Some(area) = boundary_loop.signed_area()? else {
                return Ok(None);
            };
            total = &total + &area;
        }
        Ok(Some(total))
    }

    fn native_boundary_loops(&self) -> Option<&[BezierBoundaryLoop2]> {
        self.native_boundary_loops
            .get_or_init(|| {
                self.boundary_loops
                    .iter()
                    .map(retained_loop_to_native)
                    .collect::<Option<Vec<_>>>()
                    .map(Arc::from)
            })
            .as_deref()
    }

    fn retained_rational_evaluators(&self) -> CurveResult<&[Vec<Option<RationalBezier2>>]> {
        match self.retained_rational_evaluators.get_or_init(|| {
            self.boundary_loops
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

    fn native_boundary_bounds(&self, policy: &CurvePolicy) -> Option<&[Aabb2]> {
        if let Some(bounds) = self.native_boundary_bounds.get() {
            return Some(bounds);
        }
        let native_loops = self.native_boundary_loops()?;
        let mut bounds = Vec::with_capacity(native_loops.len());
        for boundary_loop in native_loops {
            match native_loop_bounds(boundary_loop, policy) {
                Classification::Decided(boundary_bounds) => bounds.push(boundary_bounds),
                Classification::Uncertain(_) => return None,
            }
        }
        let _ = self.native_boundary_bounds.set(bounds.into());
        Some(
            self.native_boundary_bounds
                .get()
                .expect("decided native boundary bounds were retained"),
        )
    }
}

fn curve_region_loop_filled_area_magnitude(
    boundary_loop: &CurveRegionBoundaryLoop2,
    signed_area: Real,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<Real>>> {
    if let Some(period) = repeated_boundary_fragment_period(boundary_loop.fragments()) {
        let base_loop =
            CurveRegionBoundaryLoop2::new(boundary_loop.fragments()[..period].to_vec())?;
        match materialized_boundary_loop_is_simple(&base_loop, policy)? {
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
        let Some(base_area) = base_loop.signed_area()? else {
            return Ok(Classification::Decided(None));
        };
        return absolute_nonzero_area(base_area, policy).map(|area| area.map(Some));
    }

    match materialized_boundary_loop_is_simple(boundary_loop, policy)? {
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

fn absolute_nonzero_area(area: Real, policy: &CurvePolicy) -> CurveResult<Classification<Real>> {
    Ok(match real_sign(&area, policy) {
        Some(RealSign::Negative) => Classification::Decided(Real::zero() - area),
        Some(RealSign::Positive) => Classification::Decided(area),
        Some(RealSign::Zero) => Classification::Uncertain(UncertaintyReason::Boundary),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    })
}

fn materialized_boundary_loop_is_simple(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    let mut curves = Vec::with_capacity(boundary_loop.fragments().len());
    for fragment in boundary_loop.fragments() {
        let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match materialized_subcurve_has_injective_axis(curve, policy)? {
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
    let path = match CurvePath2::try_new(curves) {
        Ok(path) => path,
        Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
        Err(ExactCurveError::Blocked(blocker)) => {
            return Ok(Classification::Uncertain(blocker.reason()));
        }
    };
    let evidence = match path.intersect_path(&path, policy) {
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

fn materialized_subcurve_has_injective_axis(
    curve: &BezierSubcurve2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    let mut uncertainty = None;
    for axis in [Axis2::X, Axis2::Y] {
        let monotone = match curve {
            BezierSubcurve2::Quadratic(curve) => curve
                .axis_monotone_parameters(axis, policy)
                .map(|roots| roots.is_empty()),
            BezierSubcurve2::Cubic(curve) => curve
                .axis_monotone_parameters(axis, policy)
                .map(|roots| roots.is_empty()),
            BezierSubcurve2::RationalQuadratic(curve) => curve
                .axis_monotone_parameters(axis, policy)
                .map(|roots| roots.is_empty()),
            BezierSubcurve2::Rational(curve) => curve.axis_monotonicity_classified(axis, policy)?,
        };
        match monotone {
            Classification::Decided(true) => {
                let (start, end) = match axis {
                    Axis2::X => (curve.start().x(), curve.end().x()),
                    Axis2::Y => (curve.start().y(), curve.end().y()),
                };
                match compare_reals(start, end, policy) {
                    Some(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                        return Ok(Classification::Decided(true));
                    }
                    Some(std::cmp::Ordering::Equal) => {}
                    None => uncertainty = Some(UncertaintyReason::Ordering),
                }
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => uncertainty = Some(reason),
        }
    }
    Ok(Classification::Uncertain(
        uncertainty.unwrap_or(UncertaintyReason::Unsupported),
    ))
}

fn curve_path_contact_is_ordinary_adjacent_endpoint(
    path: &CurvePath2,
    contact: &CurvePathIntersectionContact2,
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
) -> ExactCurveResult<BezierSplitFragment2> {
    match fragment {
        BezierSplitFragment2::Materialized { start, end, curve } => {
            Ok(BezierSplitFragment2::Materialized {
                start: start.clone(),
                end: end.clone(),
                curve: transform_region_subcurve(curve, m00, m01, m10, m11, tx, ty)?,
            })
        }
        BezierSplitFragment2::AlgebraicEndpointImages {
            reversed,
            start,
            end,
            source_curve: Some(source),
            ..
        } => {
            let source = transform_region_subcurve(source, m00, m01, m10, m11, tx, ty)?;
            Ok(BezierSplitFragment2::AlgebraicEndpointImages {
                reversed: *reversed,
                start: start.clone(),
                end: end.clone(),
                start_image: transform_region_endpoint_image(start, &source, policy)?,
                end_image: transform_region_endpoint_image(end, &source, policy)?,
                source_curve: Some(source),
            })
        }
        BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: None, ..
        }
        | BezierSplitFragment2::Unresolved { .. } => Err(ExactCurveError::blocked(
            CurveOperation2::Transformation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        )),
    }
}

fn transform_region_endpoint_image(
    parameter: &BezierParameter2,
    source: &BezierSubcurve2,
    policy: &CurvePolicy,
) -> ExactCurveResult<Option<BezierAlgebraicEndpointImage2>> {
    match parameter {
        BezierParameter2::Exact(_) => Ok(None),
        BezierParameter2::Algebraic(parameter) => {
            BezierAlgebraicEndpointImage2::from_source_curve(source, parameter, policy)
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
) -> ExactCurveResult<BezierSubcurve2> {
    let point = |point: &Point2| affine_region_point(point, m00, m01, m10, m11, tx, ty);
    match curve {
        BezierSubcurve2::Quadratic(curve) => {
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
    policy: &CurvePolicy,
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
) -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    for (role, signed_area) in roles.iter().zip(signed_areas) {
        let expected = match real_sign(signed_area, &policy) {
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

fn validate_nonzero_signed_area_evidence(signed_areas: &[Real]) -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    for signed_area in signed_areas {
        match real_sign(signed_area, &policy) {
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
    policy: &CurvePolicy,
) -> CurveResult<Classification<RetainedLineLoopContour>> {
    let mut segments = Vec::with_capacity(boundary_loop.fragments().len());
    let mut materialized_fragment_count = 0_usize;
    let mut algebraic_fragment_count = 0_usize;
    for fragment in boundary_loop.fragments() {
        let endpoints = match retained_line_fragment_endpoints(fragment, policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match endpoints.source {
            RetainedLineFragmentSource::MaterializedFit => materialized_fragment_count += 1,
            RetainedLineFragmentSource::AlgebraicEndpoints => algebraic_fragment_count += 1,
        }
        let (start, end) = endpoints.points;
        segments.push(Segment2::Line(LineSeg2::try_new(start, end)?));
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
    policy: &CurvePolicy,
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
        BezierSplitFragment2::Unresolved { .. } => {
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        }
    }
}

fn subcurve_fit_exact_line_image(
    curve: &BezierSubcurve2,
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
            match exact_rational_point_from_image(image.point()) {
                Some(point) => Classification::Decided(point),
                None => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
    }
}

fn exact_rational_point_from_image(point: &BezierEndpointPointImage2) -> Option<Point2> {
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
        BezierEndpointPointImage2::Rational(point) => Some(Point2::new(
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
    }
}

struct RetainedLoopRoleDecision {
    roles: Vec<CurveRegionLoopRole>,
    nesting_depths: Vec<usize>,
}

fn retained_line_loop_roles(
    contours: &[Contour2],
    policy: &CurvePolicy,
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
    BezierBoundaryLoop2::new(fragments).ok()
}

fn native_loop_sample_point(
    boundary_loop: &BezierBoundaryLoop2,
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
        BezierSplitFragment2::Unresolved { .. } => {
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        }
    }
}

fn subcurve_control_hull_contains_point(
    curve: &BezierSubcurve2,
    point: &Point2,
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourPointLocation>> {
    if let Classification::Decided(bounds) = native_loop_bounds(boundary_loop, policy)
        && let Classification::Decided(false) = bounds.contains_point(point, policy)
    {
        return Ok(Classification::Decided(ContourPointLocation::Outside));
    }
    classify_point_against_native_loop_after_bounds(boundary_loop, point, policy)
}

fn classify_point_against_native_loop_after_bounds(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourPointLocation>> {
    if boundary_loop.fragments().len() != evaluators.len() {
        return Err(CurveError::Topology(
            "retained region evaluator cache fragment count is inconsistent".into(),
        ));
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
    policy: &CurvePolicy,
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
        BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: None, ..
        }
        | BezierSplitFragment2::Unresolved { .. } => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
    }
}

fn classify_point_with_retained_ray(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &Point2,
    ray: &BezierRay2,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourPointLocation>> {
    let direction_x = &ray.direction_x;
    let direction_y = &ray.direction_y;
    let mut winding = 0_i32;
    for fragment in boundary_loop.fragments() {
        let (curve, range) = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => (curve, None),
            BezierSplitFragment2::AlgebraicEndpointImages {
                reversed,
                start,
                end,
                source_curve: Some(curve),
                ..
            } => (curve, Some((start, end, *reversed))),
            BezierSplitFragment2::AlgebraicEndpointImages {
                source_curve: None, ..
            }
            | BezierSplitFragment2::Unresolved { .. } => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        if !subcurve_control_hull_may_be_ahead(curve, point, direction_x, direction_y, policy) {
            continue;
        }
        let control_hull_order =
            subcurve_control_hull_strict_order(curve, point, direction_x, direction_y, policy);
        let relation = match subcurve_relation_to_line_with_contacts(
            curve,
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
                    let retained = if let Some((start, end, reversed)) = range {
                        retained_parameter_contains(
                            contact.parameter(),
                            start,
                            end,
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
                            let reversed = range.is_some_and(|(_, _, reversed)| reversed);
                            let Some(delta) = line_contact_winding_delta(&contact, reversed) else {
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

fn retained_parameter_contains(
    parameter: &BezierParameter2,
    start: &BezierParameter2,
    end: &BezierParameter2,
    half_open: bool,
    reversed: bool,
    policy: &CurvePolicy,
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

fn rationalize_retained_subcurve(curve: &BezierSubcurve2) -> CurveResult<RationalBezier2> {
    let exact_line_image = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.retained_exact_line_image().cloned(),
        _ => None,
    };
    let implicit_quadratic_conic = match curve {
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.retained_implicit_quadratic_conic().cloned()
        }
        _ => None,
    };
    let circular_conic = match curve {
        BezierSubcurve2::RationalQuadratic(curve) => curve.retained_circular_conic().cloned(),
        _ => None,
    };
    let (control_points, weights) = match curve {
        BezierSubcurve2::Quadratic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 3],
        ),
        BezierSubcurve2::Cubic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 4],
        ),
        BezierSubcurve2::RationalQuadratic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            curve.weights().into_iter().cloned().collect(),
        ),
        BezierSubcurve2::Rational(curve) => return Ok(curve.clone()),
    };
    match (exact_line_image, implicit_quadratic_conic) {
        (Some(line), _) => {
            RationalBezier2::try_new_with_exact_line_image(control_points, weights, line)
        }
        (None, Some(conic)) => RationalBezier2::try_new_with_implicit_quadratic_conic(
            control_points,
            weights,
            conic,
            circular_conic,
        ),
        (None, None) => RationalBezier2::try_new(control_points, weights),
    }
}

fn native_loop_bounds(
    boundary_loop: &BezierBoundaryLoop2,
    policy: &CurvePolicy,
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

fn classify_point_with_ray(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    ray: &BezierRay2,
    fill_rule: FillRule,
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
        BezierSubcurve2::Quadratic(curve) => polynomial_image_coordinate_order(
            &curve.point_at_algebraic_parameter(parameter, policy)?,
            use_x,
            origin_coordinate,
            policy,
        ),
        BezierSubcurve2::Cubic(curve) => polynomial_image_coordinate_order(
            &curve.point_at_algebraic_parameter(parameter, policy)?,
            use_x,
            origin_coordinate,
            policy,
        ),
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
    };
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
) -> Classification<std::cmp::Ordering> {
    let coordinate = if use_x { image.x() } else { image.y() };
    coordinate.map_or(
        Classification::Uncertain(UncertaintyReason::Unsupported),
        |coordinate| coordinate.compare_to_real(origin, policy),
    )
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
fn subcurve_query_bounds(curve: &BezierSubcurve2, policy: &CurvePolicy) -> Classification<Aabb2> {
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
    policy: &CurvePolicy,
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
    pub fn signed_area_contribution(&self) -> CurveResult<Option<Real>> {
        match self {
            Self::Quadratic(curve) => curve.signed_area_contribution().map(Some),
            Self::Cubic(curve) => curve.signed_area_contribution().map(Some),
            Self::RationalQuadratic(curve) => curve.signed_area_contribution(),
            Self::Rational(curve) => match curve.signed_area_contribution()? {
                Some(area) => Ok(Some(area)),
                None => rational_line_signed_area_contribution(curve),
            },
        }
    }

    fn signed_area_contribution_with_cache(
        &self,
        rational_quadratic_cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Option<Real>> {
        match self {
            Self::Quadratic(curve) => curve.signed_area_contribution().map(Some),
            Self::Cubic(curve) => curve.signed_area_contribution().map(Some),
            Self::RationalQuadratic(curve) => {
                curve.signed_area_contribution_with_cache(rational_quadratic_cache)
            }
            Self::Rational(curve) => match curve.signed_area_contribution()? {
                Some(area) => Ok(Some(area)),
                None => rational_line_signed_area_contribution(curve),
            },
        }
    }
}

fn rational_line_signed_area_contribution(curve: &RationalBezier2) -> CurveResult<Option<Real>> {
    let Ok(line) = LineSeg2::try_new(curve.start().clone(), curve.end().clone()) else {
        return Ok(None);
    };
    if !matches!(
        curve.relation_to_line_with_contacts(&line, &CurvePolicy::certified()),
        Classification::Decided(BezierLineContactRelation::OnSupportingLine)
    ) {
        return Ok(None);
    }
    let twice_area = curve.start().x() * curve.end().y() - curve.start().y() * curve.end().x();
    Ok(Some((twice_area / Real::from(2_i8))?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cmp::Ordering;

    use crate::{
        CircularArc2, CubicBezier2, Curve2, CurvePath2, QuadraticBezier2, RationalQuadraticBezier2,
    };

    fn p(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
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
            CurveRegionBoundaryLoop2::new(fragments)
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
            CurveRegionBoundaryLoop2::new(fragments).expect("closed retained quadratic loop"),
        ])
        .expect("one retained loop")
    }

    #[test]
    fn single_loop_filled_side_uses_area_without_constructing_nesting_bounds() {
        let policy = CurvePolicy::certified();
        for (clockwise, expected) in [(false, true), (true, false)] {
            let region = single_quadratic_loop_region(clockwise);
            assert!(region.native_boundary_bounds.get().is_none());
            assert!(matches!(
                region.filled_side_is_left(&policy),
                Ok(Classification::Decided(sides)) if sides == [expected]
            ));
            assert!(region.native_boundary_bounds.get().is_none());
        }
    }

    #[test]
    fn native_query_bounds_use_exact_conservative_control_hulls() {
        let policy = CurvePolicy::certified();
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
        let policy = CurvePolicy::certified();
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
            subcurve_contains_point(&subcurve, &p(100, 0), &CurvePolicy::certified()),
            Classification::Uncertain(UncertaintyReason::Boundary)
        );
    }

    #[test]
    fn irrational_weight_semicircle_region_classifies_without_native_accelerator() {
        let policy = CurvePolicy::certified();
        let upper =
            Curve2::from(CircularArc2::try_from_center(p(0, 0), p(2, 0), p(1, 0), true).unwrap());
        let lower =
            Curve2::from(CircularArc2::try_from_center(p(2, 0), p(0, 0), p(1, 0), true).unwrap());
        let region = CurveRegion2::try_from_boundary_paths(&[CurvePath2::try_new(vec![
            upper, lower,
        ])
        .unwrap()])
        .unwrap();
        let point = Point2::new(Real::one(), (Real::one() / Real::from(2_u8)).unwrap());
        assert_eq!(
            region.classify_point(&point, &policy),
            Ok(Classification::Decided(RegionPointLocation::Inside))
        );
        assert_eq!(
            region.classify_point(&p(1, 1), &policy),
            Ok(Classification::Decided(RegionPointLocation::Boundary))
        );
        assert_eq!(
            region.classify_point(&p(1, 2), &policy),
            Ok(Classification::Decided(RegionPointLocation::Outside))
        );
        assert!(region.line_image_region.get().is_none());
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

        let policy = CurvePolicy::certified();
        let region = CurveRegion2::try_from_signed_boundary_paths_with_loop_semantics(
            &[rectangle(-3, 3), rectangle(1, 7)],
            &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole],
            &[FillRule::NonZero, FillRule::NonZero],
        )
        .unwrap();
        assert_eq!(
            region.classify_point(&p(-2, 0), &policy),
            Ok(Classification::Decided(RegionPointLocation::Inside))
        );
        assert_eq!(
            region.classify_point(&p(2, 0), &policy),
            Ok(Classification::Decided(RegionPointLocation::Outside))
        );
        assert_eq!(
            region.signed_depth(&p(2, 0), &policy),
            Ok(Classification::Decided(0))
        );
    }
}
