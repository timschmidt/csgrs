//! Exact curved regions bounded by native and algebraic curve fragments.
//!
//! [`CurveRegion2`] is the top-level higher-order region type. It accepts
//! closed [`CurvePath2`] boundaries directly and materializes decided Boolean
//! traversals without flattening their native or algebraic carriers. It
//! deliberately does not force curved boundaries into line strings or into
//! [`Region2`](crate::Region2), because Yap's exact geometric-computation
//! model requires the exact curve objects to remain visible until a certified
//! adapter exists; see Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 3-23 (1997).
//!
//! Exact area is exposed for polynomial Bezier loops and rational quadratic
//! conic loops whose homogeneous denominator is certified away from projective
//! zero on `[0, 1]`. Both use Green's-theorem boundary integrals, the same
//! identities used by [`crate::BezierAreaMoments2`]. That follows Farin's
//! Bernstein and rational Bezier identities in *Curves and Surfaces for CAGD*
//! (5th ed., 2002). Unsupported conic denominator cases still return `None`
//! rather than silently sampling.

use std::cell::OnceCell;
use std::rc::Rc;

use hyperreal::{Real, RealSign};
use hypersolve::AlgebraicRootRepresentation;

use crate::bezier_arrangement::represented_roots_equal;
use crate::classify::{compare_reals, is_zero, real_sign};
use crate::{
    Aabb2, BezierArrangementGraph2, BezierArrangementTraversal2, BezierEndpointPointImage2,
    BezierLineContactKind, BezierLineContactRelation, BezierLineImageFitRelation, BezierParameter2,
    BezierRetainedLinearOverlapTraversal2, BezierRetainedRationalOverlapTraversal2,
    BezierSplitFragment2, BezierSubcurve2, Classification, Contour2, ContourPointLocation,
    CurveError, CurveFamily2, CurveOperation2, CurvePath2, CurvePathBooleanOperand2, CurvePolicy,
    CurveResult, CurveSpanProvenance2, ExactCurveError, ExactCurveResult, LineSeg2, Point2,
    RationalBezier2, RationalBezierPointIncidence2, Region2, RegionPointLocation, Segment2,
    UncertaintyReason,
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
/// concrete exact-object region boundary in Yap's sense: the algebraic pieces
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
/// them. See Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7(1-2), 3-23 (1997), for the construction/decision separation;
/// native polynomial subloops reuse the Green-integral path described above.
#[derive(Clone, Default)]
pub struct CurveRegion2 {
    boundary_loops: Vec<CurveRegionBoundaryLoop2>,
    fragment_provenance: Option<Rc<[CurveRegionFragmentProvenance2]>>,
    filled_side_is_left: Rc<OnceCell<CurveResult<Classification<Rc<[bool]>>>>>,
    native_boundary_loops: Rc<OnceCell<Option<Rc<[BezierBoundaryLoop2]>>>>,
    native_boundary_bounds: Rc<OnceCell<Rc<[Aabb2]>>>,
    line_image_region: Rc<OnceCell<Option<Region2>>>,
    retained_rational_evaluators: Rc<OnceCell<CurveResult<Vec<Vec<Option<RationalBezier2>>>>>>,
    signed_area_cache: Rc<OnceCell<CurveResult<Option<Real>>>>,
}

impl std::fmt::Debug for CurveRegion2 {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("CurveRegion2")
            .field("boundary_loops", &self.boundary_loops)
            .field("fragment_provenance", &self.fragment_provenance)
            .finish()
    }
}

impl PartialEq for CurveRegion2 {
    fn eq(&self, other: &Self) -> bool {
        self.boundary_loops == other.boundary_loops
            && self.fragment_provenance == other.fragment_provenance
    }
}

/// Authored source lineage for one emitted top-level curved-region fragment.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionFragmentProvenance2 {
    arrangement_fragment_index: usize,
    arrangement_source_index: usize,
    operand: Option<CurvePathBooleanOperand2>,
    source_path_index: usize,
    family: CurveFamily2,
    curve_index: usize,
    promoted_span_index: usize,
    split_fragment_index: usize,
    span: CurveSpanProvenance2,
    reversed: bool,
}

impl CurveRegionFragmentProvenance2 {
    /// Constructs certified authored lineage for one emitted arrangement fragment.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        arrangement_fragment_index: usize,
        arrangement_source_index: usize,
        operand: Option<CurvePathBooleanOperand2>,
        source_path_index: usize,
        family: CurveFamily2,
        curve_index: usize,
        promoted_span_index: usize,
        split_fragment_index: usize,
        span: CurveSpanProvenance2,
        reversed: bool,
    ) -> Self {
        Self {
            arrangement_fragment_index,
            arrangement_source_index,
            operand,
            source_path_index,
            family,
            curve_index,
            promoted_span_index,
            split_fragment_index,
            span,
            reversed,
        }
    }

    /// Returns the fragment index in the emitted arrangement graph.
    pub const fn arrangement_fragment_index(&self) -> usize {
        self.arrangement_fragment_index
    }

    /// Returns the stable source index used by the arrangement graph.
    pub const fn arrangement_source_index(&self) -> usize {
        self.arrangement_source_index
    }

    /// Returns the Boolean operand, or `None` for direct boundary construction.
    pub const fn operand(&self) -> Option<CurvePathBooleanOperand2> {
        self.operand
    }

    /// Returns the source path index within the originating operation.
    pub const fn source_path_index(&self) -> usize {
        self.source_path_index
    }

    /// Returns the authored curve family.
    pub const fn family(&self) -> CurveFamily2 {
        self.family
    }

    /// Returns the authored curve index within its source path.
    pub const fn curve_index(&self) -> usize {
        self.curve_index
    }

    /// Returns the promoted native span index within the authored curve.
    pub const fn promoted_span_index(&self) -> usize {
        self.promoted_span_index
    }

    /// Returns the exact split-fragment index within the promoted span.
    pub const fn split_fragment_index(&self) -> usize {
        self.split_fragment_index
    }

    /// Returns exact authored source and parameter-range lineage.
    pub const fn span(&self) -> &CurveSpanProvenance2 {
        &self.span
    }

    /// Returns whether Boolean ownership reversed the authored traversal.
    pub const fn reversed(&self) -> bool {
        self.reversed
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

/// Exact role assignment for retained line-image Bezier boundary loops.
///
/// This report is intentionally narrower than arbitrary retained Bezier role
/// assignment.  It accepts materialized Bezier/conic fragments only through a
/// certified exact line-image fit, accepts algebraic endpoint-image fragments
/// only when they provide exact endpoint witnesses, lowers those loops to
/// native [`Contour2`] line loops, and then runs exact nesting.  This follows
/// Yap's exact-geometric-computation boundary: unsupported curve families
/// remain explicit evidence gaps rather than being sampled into polygon
/// surrogates.  The source counters retain whether role assignment consumed
/// native fit certificates or algebraic endpoint evidence.  The containment
/// step uses boundary-first point-in-contour classification as surveyed by
/// Hormann and Agathos, "The Point in Polygon Problem for Arbitrary Polygons,"
/// *Computational Geometry* 20(3), 131-144 (2001).
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionLineRoleReport2 {
    roles: Vec<CurveRegionLoopRole>,
    nesting_depths: Vec<usize>,
    materialized_fragment_count: usize,
    algebraic_fragment_count: usize,
    contours: Vec<Contour2>,
    loop_arrangement_sources: Option<Vec<Option<Vec<CurveRegionFragmentSource2>>>>,
}

/// Exact orientation-derived role assignment for native retained Bezier loops.
///
/// This report is broader than [`CurveRegionLineRoleReport2`]: it
/// accepts native polynomial Bezier and rational quadratic conic loops whenever
/// their exact Green-integral signed area is implemented and nonzero.  It is
/// intentionally narrower than full curved-loop nesting: it assigns roles from
/// the authored loop orientation only, returns the signed areas as evidence,
/// and rejects algebraic, unresolved, zero-area, or unsupported-area loops.
/// That keeps the construction/decision boundary explicit in Yap's sense; see
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7(1-2), 3-23 (1997).  The signed-area evidence comes from Green's theorem
/// and Bernstein/rational Bezier identities as described by Farin, *Curves and
/// Surfaces for CAGD* (5th ed., 2002).
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionSignedAreaRoleReport2 {
    roles: Vec<CurveRegionLoopRole>,
    signed_areas: Vec<Real>,
    loop_fragment_counts: Option<Vec<usize>>,
    loop_arrangement_sources: Option<Vec<Option<Vec<CurveRegionFragmentSource2>>>>,
}

/// Exact nesting-derived role assignment for native retained curved loops.
///
/// Unlike [`CurveRegionLineRoleReport2`], this report does not lower
/// nonlinear loops to line contours. Unlike
/// [`CurveRegionSignedAreaRoleReport2`], it does not trust authored
/// orientation to distinguish material from holes. It chooses an exact
/// representative point on each candidate loop and classifies it against every
/// other native Bezier/conic loop by counting certified ray crossings. Boundary
/// hits, tangent-only ray contacts, algebraic carriers, unresolved line-contact
/// predicates, and unsupported area/zero-area loops remain explicit
/// uncertainty. The crossing rule is the exact-object analogue of the
/// point-in-polygon method surveyed by Hormann and Agathos, "The Point in
/// Polygon Problem for Arbitrary Polygons," *Computational Geometry* 20(3),
/// 131-144 (2001); all branch decisions follow Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 3-23 (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionNestingRoleReport2 {
    roles: Vec<CurveRegionLoopRole>,
    nesting_depths: Vec<usize>,
    signed_areas: Vec<Real>,
    sample_points: Vec<Point2>,
    loop_fragment_counts: Option<Vec<usize>>,
    loop_arrangement_sources: Option<Vec<Option<Vec<CurveRegionFragmentSource2>>>>,
}

impl CurveRegionLineRoleReport2 {
    /// Constructs a retained line-image role report.
    pub fn new(
        roles: Vec<CurveRegionLoopRole>,
        nesting_depths: Vec<usize>,
        materialized_fragment_count: usize,
        algebraic_fragment_count: usize,
        contours: Vec<Contour2>,
    ) -> CurveResult<Self> {
        validate_report_length(roles.len(), "nesting depth", nesting_depths.len())?;
        validate_report_length(roles.len(), "line contour", contours.len())?;
        validate_nesting_depth_roles(&roles, &nesting_depths)?;
        validate_line_role_report_fragment_counts(
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

    /// Returns per-loop arrangement/source provenance when the report has it.
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

    /// Builds a native line-region with explicit material/hole bins.
    pub fn to_region(&self) -> Region2 {
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
        Region2::new(material, holes)
    }
}

impl CurveRegionSignedAreaRoleReport2 {
    /// Constructs a retained signed-area role report.
    pub fn new(roles: Vec<CurveRegionLoopRole>, signed_areas: Vec<Real>) -> CurveResult<Self> {
        validate_report_length(roles.len(), "signed area", signed_areas.len())?;
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

    /// Returns per-loop arrangement/source provenance when the report has it.
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

impl CurveRegionNestingRoleReport2 {
    /// Constructs a retained curved-loop nesting role report.
    pub fn new(
        roles: Vec<CurveRegionLoopRole>,
        nesting_depths: Vec<usize>,
        signed_areas: Vec<Real>,
        sample_points: Vec<Point2>,
    ) -> CurveResult<Self> {
        validate_report_length(roles.len(), "nesting depth", nesting_depths.len())?;
        validate_report_length(roles.len(), "signed area", signed_areas.len())?;
        validate_report_length(roles.len(), "sample point", sample_points.len())?;
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

    /// Returns per-loop arrangement/source provenance when the report has it.
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
        if self.fragments.is_empty() {
            return Err(CurveError::Topology(
                "Bezier boundary loop signed area requires nonempty fragments".to_owned(),
            ));
        }

        let mut total = Real::zero();
        for fragment in &self.fragments {
            let Some(contribution) = fragment.signed_area_contribution()? else {
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
    /// region materialization in Yap's exact-computation sense; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 3-23 (1997).  The positive-dimensional overlap is consumed
    /// only after the Foster, Hormann, and Popa (2019) degeneracy is recorded
    /// as a resolved span on the refinement report.
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
        Ok(Self {
            fragments,
            arrangement_sources: Some(arrangement_sources),
        })
    }

    /// Returns retained split fragments in loop order.
    pub fn fragments(&self) -> &[BezierSplitFragment2] {
        &self.fragments
    }

    /// Consumes the loop and returns retained split fragments.
    pub fn into_fragments(self) -> Vec<BezierSplitFragment2> {
        self.fragments
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
            let Some(contribution) = curve.signed_area_contribution()? else {
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

fn validate_curve_region_fragment_provenance(
    boundary_loops: &[CurveRegionBoundaryLoop2],
    provenance: &[CurveRegionFragmentProvenance2],
) -> CurveResult<()> {
    if provenance.is_empty() {
        if boundary_loops.is_empty() {
            return Ok(());
        }
        return Err(CurveError::Topology(
            "curve-region authored fragment provenance must be nonempty".into(),
        ));
    }
    for (index, source) in provenance.iter().enumerate() {
        if source.arrangement_fragment_index() != index {
            return Err(CurveError::Topology(
                "curve-region authored provenance must follow arrangement-fragment order".into(),
            ));
        }
    }

    let mut referenced = vec![false; provenance.len()];
    for boundary_loop in boundary_loops {
        let Some(sources) = boundary_loop.arrangement_sources() else {
            return Err(CurveError::Topology(
                "curve-region authored provenance requires arrangement sources on every loop"
                    .into(),
            ));
        };
        for source in sources {
            let Some(authored) = provenance.get(source.arrangement_fragment_index()) else {
                return Err(CurveError::Topology(
                    "curve-region arrangement source lacks authored provenance".into(),
                ));
            };
            if authored.arrangement_source_index() != source.source_curve_index()
                || authored.split_fragment_index() != source.source_fragment_index()
            {
                return Err(CurveError::Topology(
                    "curve-region authored provenance does not match arrangement source".into(),
                ));
            }
            referenced[source.arrangement_fragment_index()] = true;
        }
    }
    if referenced.iter().any(|referenced| !referenced) {
        return Err(CurveError::Topology(
            "curve-region authored provenance contains an unreferenced arrangement fragment".into(),
        ));
    }
    Ok(())
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

impl CurveRegion2 {
    /// Constructs a top-level exact curved region from closed boundary paths.
    ///
    /// Every authored family is promoted through its clone-shared native
    /// topology once. The result retains path, curve, promoted-span, source,
    /// and exact parameter lineage for every boundary fragment.
    pub fn try_from_boundary_paths(paths: &[CurvePath2]) -> ExactCurveResult<Self> {
        let mut boundary_loops = Vec::with_capacity(paths.len());
        let mut provenance = Vec::new();
        for (path_index, path) in paths.iter().enumerate() {
            path.bezier_boundary_loop()
                .map_err(|error| error.with_operation(CurveOperation2::Construction))?;
            let fragment_capacity = path.native_bezier_fragments()?.len();
            let mut fragments = Vec::with_capacity(fragment_capacity);
            let mut arrangement_sources = Vec::with_capacity(fragment_capacity);
            for (curve_index, curve) in path.curves().iter().enumerate() {
                for (promoted_span_index, native) in
                    curve.native_bezier_fragments()?.iter().enumerate()
                {
                    let arrangement_fragment_index = provenance.len();
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
                    provenance.push(CurveRegionFragmentProvenance2::new(
                        arrangement_fragment_index,
                        arrangement_fragment_index,
                        None,
                        path_index,
                        curve.family(),
                        curve_index,
                        promoted_span_index,
                        0,
                        native.provenance().clone(),
                        false,
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
                    path.curves()[0].source(),
                    cause,
                )
            })?;
            boundary_loops.push(boundary_loop);
        }
        Self::new(boundary_loops)
            .and_then(|region| region.with_fragment_provenance(provenance))
            .map_err(|cause| {
                let (family, source) = paths.first().map_or((CurveFamily2::Line, None), |path| {
                    (path.curves()[0].family(), path.curves()[0].source())
                });
                ExactCurveError::invalid(CurveOperation2::Construction, family, source, cause)
            })
    }

    /// Constructs an exact curved region from already materialized boundary loops.
    pub fn new(boundary_loops: Vec<CurveRegionBoundaryLoop2>) -> CurveResult<Self> {
        validate_retained_region_loops(&boundary_loops)?;
        Ok(Self {
            boundary_loops,
            fragment_provenance: None,
            filled_side_is_left: Rc::new(OnceCell::new()),
            native_boundary_loops: Rc::new(OnceCell::new()),
            native_boundary_bounds: Rc::new(OnceCell::new()),
            line_image_region: Rc::new(OnceCell::new()),
            retained_rational_evaluators: Rc::new(OnceCell::new()),
            signed_area_cache: Rc::new(OnceCell::new()),
        })
    }

    /// Attaches internally certified source lineage to every emitted fragment.
    pub(crate) fn with_fragment_provenance(
        mut self,
        fragment_provenance: Vec<CurveRegionFragmentProvenance2>,
    ) -> CurveResult<Self> {
        validate_curve_region_fragment_provenance(&self.boundary_loops, &fragment_provenance)?;
        self.fragment_provenance = Some(fragment_provenance.into());
        Ok(self)
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
            .set(Ok(Classification::Decided(Rc::from(filled_side_is_left))));
        Ok(self)
    }

    pub(crate) fn filled_side_is_left(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<&[bool]>> {
        match self
            .filled_side_is_left
            .get_or_init(|| self.compute_filled_side_is_left(policy))
        {
            Ok(Classification::Decided(sides)) => Ok(Classification::Decided(sides.as_ref())),
            Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(*reason)),
            Err(error) => Err(error.clone()),
        }
    }

    fn compute_filled_side_is_left(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Rc<[bool]>>> {
        match self.curved_nesting_role_report(policy)? {
            Classification::Decided(report) => {
                return filled_sides_from_roles_and_areas(
                    report.roles(),
                    report.signed_areas(),
                    policy,
                )
                .map(|sides| Classification::Decided(Rc::from(sides)));
            }
            Classification::Uncertain(UncertaintyReason::Unsupported) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        match self.line_image_role_report(policy)? {
            Classification::Decided(report) => {
                let mut areas = Vec::with_capacity(report.contours().len());
                for contour in report.contours() {
                    let Some(area) = contour.signed_area()? else {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    };
                    areas.push(area);
                }
                filled_sides_from_roles_and_areas(report.roles(), &areas, policy)
                    .map(|sides| Classification::Decided(Rc::from(sides)))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Returns authored source lineage for direct or Boolean construction.
    pub fn fragment_provenance(&self) -> Option<&[CurveRegionFragmentProvenance2]> {
        self.fragment_provenance.as_deref()
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
            if validate_retained_arrangement_chain_connectivity(
                graph,
                chain.fragment_indices(),
                &CurvePolicy::certified(),
            )
            .is_err()
            {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            let loop_ = match CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                fragments,
                arrangement_sources,
            ) {
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
    pub fn line_image_role_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveRegionLineRoleReport2>> {
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
        let report = CurveRegionLineRoleReport2::new(
            roles.roles,
            roles.nesting_depths,
            materialized_fragment_count,
            algebraic_fragment_count,
            contours,
        )?
        .with_loop_arrangement_sources(retained_loop_arrangement_sources(&self.boundary_loops))?;
        Ok(Classification::Decided(report))
    }

    /// Assigns material/hole roles from exact native loop signed-area orientation.
    ///
    /// A negative signed area is treated as a material loop and a positive
    /// signed area as a hole loop, matching the current Bezier region boundary
    /// convention used by [`BezierRegion2::signed_area`].  This method is a
    /// report-bearing orientation adapter: it does not infer nesting and it
    /// does not sample nonlinear loops.  Use [`Self::line_image_role_report`]
    /// when exact line-image nesting is required.
    pub fn signed_area_role_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveRegionSignedAreaRoleReport2>> {
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
        let report = CurveRegionSignedAreaRoleReport2::new(roles, signed_areas)?
            .with_loop_fragment_counts(retained_loop_fragment_counts(&self.boundary_loops))?
            .with_loop_arrangement_sources(retained_loop_arrangement_sources(
                &self.boundary_loops,
            ))?;
        Ok(Classification::Decided(report))
    }

    /// Assigns material/hole roles by exact curved-loop nesting.
    ///
    /// Each retained loop must be fully native and have a nonzero implemented
    /// signed area. The area is used only to reject degenerate/unsupported
    /// loops; role parity comes from exact containment depth. This makes
    /// same-orientation nested nonlinear loops classify as material/hole by
    /// topology instead of by their authored orientation.
    pub fn curved_nesting_role_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveRegionNestingRoleReport2>> {
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

        let report =
            CurveRegionNestingRoleReport2::new(roles, nesting_depths, signed_areas, sample_points)?
                .with_loop_fragment_counts(retained_loop_fragment_counts(&self.boundary_loops))?
                .with_loop_arrangement_sources(retained_loop_arrangement_sources(
                    &self.boundary_loops,
                ))?;
        Ok(Classification::Decided(report))
    }

    /// Returns whether native boundary conversion has already been retained.
    pub fn is_native_boundary_cache_cached(&self) -> bool {
        self.native_boundary_loops.get().is_some()
    }

    /// Returns whether decided native boundary bounds have been retained.
    pub fn is_native_boundary_bounds_cache_cached(&self) -> bool {
        self.native_boundary_bounds.get().is_some()
    }

    /// Returns whether line-image eligibility and any decided region have been cached.
    pub fn is_line_image_region_cached(&self) -> bool {
        self.line_image_region.get().is_some()
    }

    /// Returns whether algebraic-carrier rational evaluators have been retained.
    pub fn is_retained_rational_evaluator_cache_cached(&self) -> bool {
        self.retained_rational_evaluators.get().is_some()
    }

    /// Returns whether the exact aggregate signed area has been retained.
    pub fn is_signed_area_cached(&self) -> bool {
        self.signed_area_cache.get().is_some()
    }

    /// Classifies a point against the exact retained region using even-odd fill.
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
        let Some(native_loops) = self.native_boundary_loops() else {
            if let Some(region) = self.line_image_region.get() {
                return match region {
                    Some(region) => Ok(region.classify_point(point, policy)),
                    None => classify_point_against_retained_loops(
                        &self.boundary_loops,
                        self.retained_rational_evaluators()?,
                        point,
                        policy,
                    ),
                };
            }
            return match self.line_image_role_report(policy)? {
                Classification::Decided(report) => {
                    let _ = self.line_image_region.set(Some(report.to_region()));
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
                    )
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        };
        let native_bounds = self.native_boundary_bounds(policy);
        let mut inside = false;
        for (index, boundary_loop) in native_loops.iter().enumerate() {
            if native_bounds.is_some_and(|bounds| {
                matches!(
                    bounds[index].contains_point(point, policy),
                    Classification::Decided(false)
                )
            }) {
                continue;
            }
            match classify_point_against_native_loop_after_bounds(boundary_loop, point, policy)? {
                Classification::Decided(ContourPointLocation::Inside) => inside = !inside,
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(Classification::Decided(RegionPointLocation::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(if inside {
            RegionPointLocation::Inside
        } else {
            RegionPointLocation::Outside
        }))
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
                    .map(Rc::from)
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
    validate_report_length(
        loop_count,
        "loop fragment count",
        loop_fragment_counts.len(),
    )?;
    if loop_fragment_counts.contains(&0) {
        return Err(CurveError::Topology(
            "retained role report loop fragment counts must be nonzero".into(),
        ));
    }
    Ok(())
}

fn validate_loop_arrangement_sources(
    loop_count: usize,
    loop_arrangement_sources: &[Option<Vec<CurveRegionFragmentSource2>>],
) -> CurveResult<()> {
    validate_report_length(
        loop_count,
        "loop arrangement source",
        loop_arrangement_sources.len(),
    )?;
    if loop_arrangement_sources.iter().flatten().any(Vec::is_empty) {
        return Err(CurveError::Topology(
            "retained role report present loop arrangement sources must be nonempty".into(),
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
        "retained role report loop arrangement sources must not reuse arrangement fragments",
    )
}

fn validate_counted_loop_arrangement_source_counts(
    loop_fragment_counts: Option<&[usize]>,
    loop_arrangement_sources: &[Option<Vec<CurveRegionFragmentSource2>>],
) -> CurveResult<()> {
    let Some(loop_fragment_counts) = loop_fragment_counts else {
        if loop_arrangement_sources.iter().any(Option::is_some) {
            return Err(CurveError::Topology(
                "retained role report present loop arrangement sources require loop fragment count evidence"
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
                "retained role report loop source count does not match loop fragment count".into(),
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

fn validate_report_length(
    loop_count: usize,
    evidence_name: &str,
    evidence_count: usize,
) -> CurveResult<()> {
    if loop_count == 0 {
        return Err(CurveError::Topology(
            "retained role report must carry at least one loop".into(),
        ));
    }
    if loop_count != evidence_count {
        return Err(CurveError::Topology(format!(
            "retained role report {evidence_name} count does not match loop count"
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
                "retained nesting role report role does not match certified nesting depth".into(),
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
                    "retained signed-area role report must carry certified nonzero area evidence"
                        .into(),
                ));
            }
        };
        if *role != expected {
            return Err(CurveError::Topology(
                "retained signed-area role report role does not match signed-area evidence".into(),
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
                    "retained curved nesting role report must carry certified nonzero signed-area evidence"
                        .into(),
                ));
            }
        }
    }
    Ok(())
}

fn validate_line_role_report_fragment_counts(
    materialized_fragment_count: usize,
    algebraic_fragment_count: usize,
    contours: &[Contour2],
) -> CurveResult<()> {
    let source_fragment_count = materialized_fragment_count
        .checked_add(algebraic_fragment_count)
        .ok_or_else(|| {
            CurveError::Topology(
                "retained line role report source fragment count overflowed".into(),
            )
        })?;
    let contour_fragment_count = contours
        .iter()
        .try_fold(0_usize, |count, contour| count.checked_add(contour.len()))
        .ok_or_else(|| {
            CurveError::Topology(
                "retained line role report contour fragment count overflowed".into(),
            )
        })?;
    if source_fragment_count != contour_fragment_count {
        return Err(CurveError::Topology(
            "retained line role report source fragment count does not match line contour evidence"
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
                "retained line role report loop source count does not match contour fragment count"
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
/// curve. This follows Yap's retained-object discipline: algebraic endpoints
/// become line-contour topology only through exact construction evidence, not
/// by sampling isolating intervals. The native fit certificate proves every
/// control point lies on the endpoint segment, preserving the exact
/// object/predicate split described by Yap while allowing non-affine
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
    for fragment in boundary_loop.fragments() {
        match subcurve_contains_point(fragment, point, policy) {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    let rays = ray_candidates(point);
    let mut last_reason = UncertaintyReason::Boundary;
    for ray in rays {
        match classify_point_with_ray(boundary_loop, point, &ray, policy)? {
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
) -> CurveResult<Classification<RegionPointLocation>> {
    if boundary_loops.len() != evaluators.len() {
        return Err(CurveError::Topology(
            "retained region evaluator cache loop count is inconsistent".into(),
        ));
    }
    let mut inside = false;
    for (boundary_loop, evaluators) in boundary_loops.iter().zip(evaluators) {
        match classify_point_against_retained_loop(boundary_loop, evaluators, point, policy)? {
            Classification::Decided(ContourPointLocation::Inside) => inside = !inside,
            Classification::Decided(ContourPointLocation::Outside) => {}
            Classification::Decided(ContourPointLocation::Boundary) => {
                return Ok(Classification::Decided(RegionPointLocation::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
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
    if boundary_loop.fragments().len() != evaluators.len() {
        return Err(CurveError::Topology(
            "retained region evaluator cache fragment count is inconsistent".into(),
        ));
    }
    for (fragment, evaluator) in boundary_loop.fragments().iter().zip(evaluators) {
        match retained_fragment_contains_point(fragment, evaluator.as_ref(), point, policy)? {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    let mut last_reason = UncertaintyReason::Boundary;
    for ray in ray_candidates(point) {
        match classify_point_with_retained_ray(boundary_loop, point, &ray, policy)? {
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
    ray: &LineSeg2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourPointLocation>> {
    let direction_x = ray.end().x() - ray.start().x();
    let direction_y = ray.end().y() - ray.start().y();
    let mut crossings = 0_usize;
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
        let relation = match subcurve_relation_to_line_with_contacts(curve, ray, policy) {
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
                    if contact.kind() != BezierLineContactKind::Crossing {
                        continue;
                    }
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
                    let ahead = match contact.parameter() {
                        BezierParameter2::Exact(parameter) => {
                            let contact_point =
                                match subcurve_point_at(curve, parameter.clone(), policy) {
                                    Classification::Decided(point) => point,
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                };
                            let projection = (contact_point.x() - point.x()) * &direction_x
                                + (contact_point.y() - point.y()) * &direction_y;
                            compare_reals(&projection, &Real::zero(), policy)
                                .map(Classification::Decided)
                                .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign))
                        }
                        BezierParameter2::Algebraic(parameter) => {
                            algebraic_contact_order_along_ray(
                                curve,
                                parameter,
                                point,
                                &direction_x,
                                &direction_y,
                                policy,
                            )?
                        }
                    };
                    match ahead {
                        Classification::Decided(std::cmp::Ordering::Greater) => crossings += 1,
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
    Ok(Classification::Decided(if crossings.is_multiple_of(2) {
        ContourPointLocation::Outside
    } else {
        ContourPointLocation::Inside
    }))
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
    RationalBezier2::try_new(control_points, weights)
}

fn native_loop_bounds(
    boundary_loop: &BezierBoundaryLoop2,
    policy: &CurvePolicy,
) -> Classification<Aabb2> {
    let Some(first) = boundary_loop.fragments().first() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let mut bounds = match subcurve_bounds(first, policy) {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    for fragment in &boundary_loop.fragments()[1..] {
        let fragment_bounds = match subcurve_bounds(fragment, policy) {
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
    ray: &LineSeg2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourPointLocation>> {
    let direction_x = ray.end().x() - ray.start().x();
    let direction_y = ray.end().y() - ray.start().y();
    let mut crossings = 0_usize;
    for fragment in boundary_loop.fragments() {
        let relation = match subcurve_relation_to_line_with_contacts(fragment, ray, policy) {
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
                    if contact.kind() != BezierLineContactKind::Crossing {
                        continue;
                    }
                    let one = BezierParameter2::Exact(Real::one());
                    match contact.parameter().cmp_by_interval(&one, policy)? {
                        Classification::Decided(std::cmp::Ordering::Equal) => continue,
                        Classification::Decided(_) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                    let ahead = match contact.parameter() {
                        BezierParameter2::Exact(parameter) => {
                            let contact_point =
                                match subcurve_point_at(fragment, parameter.clone(), policy) {
                                    Classification::Decided(point) => point,
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                };
                            let projection = (contact_point.x() - point.x()) * &direction_x
                                + (contact_point.y() - point.y()) * &direction_y;
                            compare_reals(&projection, &Real::zero(), policy)
                                .map(Classification::Decided)
                                .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign))
                        }
                        BezierParameter2::Algebraic(parameter) => {
                            algebraic_contact_order_along_ray(
                                fragment,
                                parameter,
                                point,
                                &direction_x,
                                &direction_y,
                                policy,
                            )?
                        }
                    };
                    match ahead {
                        Classification::Decided(std::cmp::Ordering::Greater) => crossings += 1,
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

    Ok(Classification::Decided(if crossings.is_multiple_of(2) {
        ContourPointLocation::Outside
    } else {
        ContourPointLocation::Inside
    }))
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

fn ray_candidates(point: &Point2) -> Vec<LineSeg2> {
    let one = Real::one();
    let two = Real::from(2_i8);
    let endpoints = [
        Point2::new(point.x() + &one, point.y().clone()),
        Point2::new(point.x().clone(), point.y() + &one),
        Point2::new(point.x() + &one, point.y() + &two),
        Point2::new(point.x() + &two, point.y() + &one),
    ];
    endpoints
        .into_iter()
        .map(|endpoint| {
            LineSeg2::try_new(point.clone(), endpoint)
                .expect("fixed exact ray directions are nonzero")
        })
        .collect()
}

fn subcurve_bounds(curve: &BezierSubcurve2, policy: &CurvePolicy) -> Classification<Aabb2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Cubic(curve) => curve.certified_bounds(policy),
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
    policy: &CurvePolicy,
) -> Classification<BezierLineContactRelation> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.relation_to_line_with_contacts(line, policy),
        BezierSubcurve2::Cubic(curve) => curve.relation_to_line_with_contacts(line, policy),
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
            Self::Rational(curve) => {
                let Ok(line) = LineSeg2::try_new(curve.start().clone(), curve.end().clone()) else {
                    return Ok(None);
                };
                if !matches!(
                    curve.relation_to_line_with_contacts(&line, &CurvePolicy::certified()),
                    Classification::Decided(BezierLineContactRelation::OnSupportingLine)
                ) {
                    return Ok(None);
                }
                let twice_area =
                    curve.start().x() * curve.end().y() - curve.start().y() * curve.end().x();
                Ok(Some((twice_area / Real::from(2_i8))?))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RationalQuadraticBezier2;

    fn p(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
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
}
