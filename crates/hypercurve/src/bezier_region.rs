//! Concrete retained regions bounded by Bezier and conic fragments.
//!
//! This module is the first higher-order region materializer: it consumes a
//! decided retained arrangement traversal and emits closed loops of native
//! polynomial Bezier and rational quadratic conic fragments. It deliberately
//! does not flatten these boundaries to line strings or force them into
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

use hyperreal::{Real, RealSign};

use crate::classify::{compare_reals, real_sign};
use crate::{
    Aabb2, BezierArrangementGraph2, BezierArrangementTraversal2, BezierEndpointPointImage2,
    BezierLineContactKind, BezierLineContactRelation, BezierLineImageFitRelation, BezierParameter2,
    BezierRetainedLinearOverlapTraversal2, BezierSplitFragment2, BezierSubcurve2, Classification,
    Contour2, ContourPointLocation, CurveError, CurvePolicy, CurveResult, LineSeg2, Point2,
    Region2, Segment2, UncertaintyReason,
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
pub struct BezierRetainedBoundaryLoop2 {
    fragments: Vec<BezierSplitFragment2>,
    arrangement_sources: Option<Vec<BezierRetainedFragmentSource2>>,
}

/// Arrangement provenance for one retained boundary fragment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BezierRetainedFragmentSource2 {
    arrangement_fragment_index: usize,
    source_curve_index: usize,
    source_fragment_index: usize,
}

impl BezierRetainedFragmentSource2 {
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
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BezierRetainedRegion2 {
    boundary_loops: Vec<BezierRetainedBoundaryLoop2>,
}

/// Material/hole role assigned to one retained Bezier boundary loop.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierRetainedRegionLoopRole {
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
pub struct BezierRetainedLineRegionRoleReport2 {
    roles: Vec<BezierRetainedRegionLoopRole>,
    nesting_depths: Vec<usize>,
    materialized_fragment_count: usize,
    algebraic_fragment_count: usize,
    contours: Vec<Contour2>,
    loop_arrangement_sources: Option<Vec<Option<Vec<BezierRetainedFragmentSource2>>>>,
}

/// Exact orientation-derived role assignment for native retained Bezier loops.
///
/// This report is broader than [`BezierRetainedLineRegionRoleReport2`]: it
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
pub struct BezierRetainedSignedAreaRoleReport2 {
    roles: Vec<BezierRetainedRegionLoopRole>,
    signed_areas: Vec<Real>,
    loop_arrangement_sources: Option<Vec<Option<Vec<BezierRetainedFragmentSource2>>>>,
}

/// Exact nesting-derived role assignment for native retained curved loops.
///
/// Unlike [`BezierRetainedLineRegionRoleReport2`], this report does not lower
/// nonlinear loops to line contours. Unlike
/// [`BezierRetainedSignedAreaRoleReport2`], it does not trust authored
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
pub struct BezierRetainedCurvedNestingRoleReport2 {
    roles: Vec<BezierRetainedRegionLoopRole>,
    nesting_depths: Vec<usize>,
    signed_areas: Vec<Real>,
    sample_points: Vec<Point2>,
    loop_arrangement_sources: Option<Vec<Option<Vec<BezierRetainedFragmentSource2>>>>,
}

impl BezierRetainedLineRegionRoleReport2 {
    /// Constructs a retained line-image role report.
    pub const fn new(
        roles: Vec<BezierRetainedRegionLoopRole>,
        nesting_depths: Vec<usize>,
        materialized_fragment_count: usize,
        algebraic_fragment_count: usize,
        contours: Vec<Contour2>,
    ) -> Self {
        Self {
            roles,
            nesting_depths,
            materialized_fragment_count,
            algebraic_fragment_count,
            contours,
            loop_arrangement_sources: None,
        }
    }

    /// Attaches one optional arrangement source trail per retained loop.
    pub fn with_loop_arrangement_sources(
        mut self,
        loop_arrangement_sources: Vec<Option<Vec<BezierRetainedFragmentSource2>>>,
    ) -> CurveResult<Self> {
        validate_loop_arrangement_sources(self.roles.len(), &loop_arrangement_sources)?;
        self.loop_arrangement_sources = Some(loop_arrangement_sources);
        Ok(self)
    }

    /// Returns one assigned role per retained boundary loop.
    pub fn roles(&self) -> &[BezierRetainedRegionLoopRole] {
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
    pub fn loop_arrangement_sources(
        &self,
    ) -> Option<&[Option<Vec<BezierRetainedFragmentSource2>>]> {
        self.loop_arrangement_sources.as_deref()
    }

    /// Returns loop indices assigned as material.
    pub fn material_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| {
                (*role == BezierRetainedRegionLoopRole::Material).then_some(index)
            })
            .collect()
    }

    /// Returns loop indices assigned as holes.
    pub fn hole_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| {
                (*role == BezierRetainedRegionLoopRole::Hole).then_some(index)
            })
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
                BezierRetainedRegionLoopRole::Material => material.push(contour),
                BezierRetainedRegionLoopRole::Hole => holes.push(contour),
            }
        }
        Region2::new(material, holes)
    }
}

impl BezierRetainedSignedAreaRoleReport2 {
    /// Constructs a retained signed-area role report.
    pub const fn new(roles: Vec<BezierRetainedRegionLoopRole>, signed_areas: Vec<Real>) -> Self {
        Self {
            roles,
            signed_areas,
            loop_arrangement_sources: None,
        }
    }

    /// Attaches one optional arrangement source trail per retained loop.
    pub fn with_loop_arrangement_sources(
        mut self,
        loop_arrangement_sources: Vec<Option<Vec<BezierRetainedFragmentSource2>>>,
    ) -> CurveResult<Self> {
        validate_loop_arrangement_sources(self.roles.len(), &loop_arrangement_sources)?;
        self.loop_arrangement_sources = Some(loop_arrangement_sources);
        Ok(self)
    }

    /// Returns one assigned role per retained boundary loop.
    pub fn roles(&self) -> &[BezierRetainedRegionLoopRole] {
        &self.roles
    }

    /// Returns exact signed areas used as orientation evidence.
    pub fn signed_areas(&self) -> &[Real] {
        &self.signed_areas
    }

    /// Returns per-loop arrangement/source provenance when the report has it.
    pub fn loop_arrangement_sources(
        &self,
    ) -> Option<&[Option<Vec<BezierRetainedFragmentSource2>>]> {
        self.loop_arrangement_sources.as_deref()
    }

    /// Returns loop indices assigned as material.
    pub fn material_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| {
                (*role == BezierRetainedRegionLoopRole::Material).then_some(index)
            })
            .collect()
    }

    /// Returns loop indices assigned as holes.
    pub fn hole_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| {
                (*role == BezierRetainedRegionLoopRole::Hole).then_some(index)
            })
            .collect()
    }
}

impl BezierRetainedCurvedNestingRoleReport2 {
    /// Constructs a retained curved-loop nesting role report.
    pub const fn new(
        roles: Vec<BezierRetainedRegionLoopRole>,
        nesting_depths: Vec<usize>,
        signed_areas: Vec<Real>,
        sample_points: Vec<Point2>,
    ) -> Self {
        Self {
            roles,
            nesting_depths,
            signed_areas,
            sample_points,
            loop_arrangement_sources: None,
        }
    }

    /// Attaches one optional arrangement source trail per retained loop.
    pub fn with_loop_arrangement_sources(
        mut self,
        loop_arrangement_sources: Vec<Option<Vec<BezierRetainedFragmentSource2>>>,
    ) -> CurveResult<Self> {
        validate_loop_arrangement_sources(self.roles.len(), &loop_arrangement_sources)?;
        self.loop_arrangement_sources = Some(loop_arrangement_sources);
        Ok(self)
    }

    /// Returns one assigned role per retained boundary loop.
    pub fn roles(&self) -> &[BezierRetainedRegionLoopRole] {
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
    pub fn loop_arrangement_sources(
        &self,
    ) -> Option<&[Option<Vec<BezierRetainedFragmentSource2>>]> {
        self.loop_arrangement_sources.as_deref()
    }

    /// Returns loop indices assigned as material.
    pub fn material_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| {
                (*role == BezierRetainedRegionLoopRole::Material).then_some(index)
            })
            .collect()
    }

    /// Returns loop indices assigned as holes.
    pub fn hole_loop_indices(&self) -> Vec<usize> {
        self.roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| {
                (*role == BezierRetainedRegionLoopRole::Hole).then_some(index)
            })
            .collect()
    }
}

impl BezierBoundaryLoop2 {
    /// Constructs a closed boundary loop from native Bezier/conic fragments.
    pub const fn new(fragments: Vec<BezierSubcurve2>) -> Self {
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
    pub fn signed_area(&self) -> CurveResult<Option<Real>> {
        let mut total = Real::zero();
        for fragment in &self.fragments {
            let Some(contribution) = fragment.signed_area_contribution()? else {
                return Ok(None);
            };
            total = &total + &contribution;
        }
        Ok(Some(total))
    }
}

impl BezierRegion2 {
    /// Constructs a retained region from closed boundary loops.
    pub const fn new(boundary_loops: Vec<BezierBoundaryLoop2>) -> Self {
        Self { boundary_loops }
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
            loops.push(BezierBoundaryLoop2::new(fragments));
        }

        Classification::Decided(Self::new(loops))
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

impl BezierRetainedBoundaryLoop2 {
    /// Constructs a retained boundary loop from accepted split fragments.
    pub const fn new(fragments: Vec<BezierSplitFragment2>) -> Self {
        Self {
            fragments,
            arrangement_sources: None,
        }
    }

    /// Constructs a retained boundary loop with one source record per fragment.
    pub fn try_new_with_arrangement_sources(
        fragments: Vec<BezierSplitFragment2>,
        arrangement_sources: Vec<BezierRetainedFragmentSource2>,
    ) -> CurveResult<Self> {
        if fragments.len() != arrangement_sources.len() {
            return Err(CurveError::Topology(
                "retained boundary source count does not match fragment count".to_owned(),
            ));
        }
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
    pub fn arrangement_sources(&self) -> Option<&[BezierRetainedFragmentSource2]> {
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

impl BezierRetainedRegion2 {
    /// Constructs a retained region from retained boundary loops.
    pub const fn new(boundary_loops: Vec<BezierRetainedBoundaryLoop2>) -> Self {
        Self { boundary_loops }
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
                arrangement_sources.push(BezierRetainedFragmentSource2::new(
                    *index,
                    fragment.source_curve_index(),
                    fragment.source_fragment_index(),
                ));
            }
            let loop_ = match BezierRetainedBoundaryLoop2::try_new_with_arrangement_sources(
                fragments,
                arrangement_sources,
            ) {
                Ok(loop_) => loop_,
                Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
            };
            loops.push(loop_);
        }

        Classification::Decided(Self::new(loops))
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
    ) -> CurveResult<Classification<BezierRetainedLineRegionRoleReport2>> {
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
        let report = BezierRetainedLineRegionRoleReport2::new(
            roles.roles,
            roles.nesting_depths,
            materialized_fragment_count,
            algebraic_fragment_count,
            contours,
        )
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
    ) -> CurveResult<Classification<BezierRetainedSignedAreaRoleReport2>> {
        let mut roles = Vec::with_capacity(self.boundary_loops.len());
        let mut signed_areas = Vec::with_capacity(self.boundary_loops.len());
        for boundary_loop in &self.boundary_loops {
            let Some(area) = boundary_loop.signed_area()? else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let role = match real_sign(&area, policy) {
                Some(RealSign::Negative) => BezierRetainedRegionLoopRole::Material,
                Some(RealSign::Positive) => BezierRetainedRegionLoopRole::Hole,
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            roles.push(role);
            signed_areas.push(area);
        }
        let report = BezierRetainedSignedAreaRoleReport2::new(roles, signed_areas)
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
    ) -> CurveResult<Classification<BezierRetainedCurvedNestingRoleReport2>> {
        let mut native_loops = Vec::with_capacity(self.boundary_loops.len());
        let mut sample_points = Vec::with_capacity(self.boundary_loops.len());
        let mut signed_areas = Vec::with_capacity(self.boundary_loops.len());
        for boundary_loop in &self.boundary_loops {
            let native_loop = match retained_loop_to_native(boundary_loop) {
                Some(loop_) => loop_,
                None => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            };
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
            let sample = match native_loop_sample_point(&native_loop, policy) {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            sample_points.push(sample);
            signed_areas.push(area);
            native_loops.push(native_loop);
        }

        let mut roles = Vec::with_capacity(native_loops.len());
        let mut nesting_depths = Vec::with_capacity(native_loops.len());
        for (candidate_index, sample) in sample_points.iter().enumerate() {
            let mut depth = 0_usize;
            for (container_index, container) in native_loops.iter().enumerate() {
                if candidate_index == container_index {
                    continue;
                }
                match classify_point_against_native_loop(container, sample, policy)? {
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
                BezierRetainedRegionLoopRole::Material
            } else {
                BezierRetainedRegionLoopRole::Hole
            });
        }

        let report = BezierRetainedCurvedNestingRoleReport2::new(
            roles,
            nesting_depths,
            signed_areas,
            sample_points,
        )
        .with_loop_arrangement_sources(retained_loop_arrangement_sources(&self.boundary_loops))?;
        Ok(Classification::Decided(report))
    }

    /// Returns retained boundary loops.
    pub fn boundary_loops(&self) -> &[BezierRetainedBoundaryLoop2] {
        &self.boundary_loops
    }

    /// Consumes the region and returns retained boundary loops.
    pub fn into_boundary_loops(self) -> Vec<BezierRetainedBoundaryLoop2> {
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
            .any(BezierRetainedBoundaryLoop2::has_algebraic_fragments)
    }

    /// Returns exact signed area only when all retained loops are native
    /// polynomial loops with implemented Green integrals.
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

struct RetainedLineLoopContour {
    contour: Contour2,
    materialized_fragment_count: usize,
    algebraic_fragment_count: usize,
}

fn retained_loop_arrangement_sources(
    boundary_loops: &[BezierRetainedBoundaryLoop2],
) -> Vec<Option<Vec<BezierRetainedFragmentSource2>>> {
    boundary_loops
        .iter()
        .map(|boundary_loop| boundary_loop.arrangement_sources().map(<[_]>::to_vec))
        .collect()
}

fn validate_loop_arrangement_sources(
    loop_count: usize,
    loop_arrangement_sources: &[Option<Vec<BezierRetainedFragmentSource2>>],
) -> CurveResult<()> {
    if loop_count != loop_arrangement_sources.len() {
        return Err(CurveError::Topology(
            "retained role report source count does not match loop count".to_owned(),
        ));
    }
    Ok(())
}

fn retained_line_loop_to_contour(
    boundary_loop: &BezierRetainedBoundaryLoop2,
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
            Ok(Classification::Decided(RetainedLineFragmentEndpoints {
                points: (start, end),
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
        BezierEndpointPointImage2::RationalQuadratic(point) => Some(Point2::new(
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
    roles: Vec<BezierRetainedRegionLoopRole>,
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
            BezierRetainedRegionLoopRole::Material
        } else {
            BezierRetainedRegionLoopRole::Hole
        });
    }
    Ok(Classification::Decided(RetainedLoopRoleDecision {
        roles,
        nesting_depths,
    }))
}

fn retained_loop_to_native(
    boundary_loop: &BezierRetainedBoundaryLoop2,
) -> Option<BezierBoundaryLoop2> {
    let mut fragments = Vec::with_capacity(boundary_loop.fragments().len());
    for fragment in boundary_loop.fragments() {
        let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
            return None;
        };
        fragments.push(curve.clone());
    }
    Some(BezierBoundaryLoop2::new(fragments))
}

fn native_loop_sample_point(
    boundary_loop: &BezierBoundaryLoop2,
    policy: &CurvePolicy,
) -> Classification<Point2> {
    let Some(fragment) = boundary_loop.fragments().first() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let half =
        (Real::one() / Real::from(2_i8)).expect("division by positive integer constant is defined");
    subcurve_point_at(fragment, half, policy)
}

fn classify_point_against_native_loop(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourPointLocation>> {
    let ray = match horizontal_ray_past_loop(boundary_loop, point, policy) {
        Classification::Decided(ray) => ray,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut crossings = 0_usize;
    for fragment in boundary_loop.fragments() {
        if subcurve_contains_point(fragment, point, policy)? {
            return Ok(Classification::Decided(ContourPointLocation::Boundary));
        }
        let relation = match subcurve_relation_to_line_with_contacts(fragment, &ray, policy) {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match relation {
            BezierLineContactRelation::ControlHullDisjoint { .. } => {}
            BezierLineContactRelation::OnSupportingLine => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            BezierLineContactRelation::Contacts { contacts } => {
                for contact in contacts {
                    if contact.kind() != BezierLineContactKind::Crossing {
                        continue;
                    }
                    let parameter_cmp = compare_reals(contact.parameter(), &Real::one(), policy);
                    if matches!(parameter_cmp, Some(std::cmp::Ordering::Equal)) {
                        continue;
                    }
                    if parameter_cmp.is_none() {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                    let contact_point =
                        match subcurve_point_at(fragment, contact.parameter().clone(), policy) {
                            Classification::Decided(point) => point,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    match compare_reals(contact_point.x(), point.x(), policy) {
                        Some(std::cmp::Ordering::Greater) => crossings += 1,
                        Some(std::cmp::Ordering::Equal) => {
                            return Ok(Classification::Decided(ContourPointLocation::Boundary));
                        }
                        Some(std::cmp::Ordering::Less) => {}
                        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                    }
                }
            }
            BezierLineContactRelation::IsolatedIntersections { .. }
            | BezierLineContactRelation::Unresolved => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        }
    }

    Ok(Classification::Decided(if crossings.is_multiple_of(2) {
        ContourPointLocation::Outside
    } else {
        ContourPointLocation::Inside
    }))
}

fn horizontal_ray_past_loop(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<LineSeg2> {
    let mut right_x = point.x() + Real::one();
    for fragment in boundary_loop.fragments() {
        let bounds = match subcurve_bounds(fragment, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        if matches!(
            compare_reals(bounds.max_x(), &right_x, policy),
            Some(std::cmp::Ordering::Greater)
        ) {
            right_x = bounds.max_x() + Real::one();
        } else if compare_reals(bounds.max_x(), &right_x, policy).is_none() {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }
    }
    match LineSeg2::try_new(point.clone(), Point2::new(right_x, point.y().clone())) {
        Ok(ray) => Classification::Decided(ray),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn subcurve_bounds(curve: &BezierSubcurve2, policy: &CurvePolicy) -> Classification<Aabb2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Cubic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(policy),
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
    }
}

fn subcurve_contains_point(
    curve: &BezierSubcurve2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<bool> {
    let classification = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.contains_point(point, policy),
        BezierSubcurve2::Cubic(_) => Classification::Decided(false),
        BezierSubcurve2::RationalQuadratic(curve) => curve.contains_point(point, policy),
    };
    match classification {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(UncertaintyReason::Unsupported) => Ok(false),
        Classification::Uncertain(reason) => Err(crate::CurveError::Topology(format!(
            "could not certify retained curved-loop boundary point query: {reason:?}"
        ))),
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
    }
}

impl BezierSubcurve2 {
    /// Returns exact signed-area contribution when implemented for this curve family.
    pub fn signed_area_contribution(&self) -> CurveResult<Option<Real>> {
        match self {
            Self::Quadratic(curve) => curve.signed_area_contribution().map(Some),
            Self::Cubic(curve) => curve.signed_area_contribution().map(Some),
            Self::RationalQuadratic(curve) => curve.signed_area_contribution(),
        }
    }
}
