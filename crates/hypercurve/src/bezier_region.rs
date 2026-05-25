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
//! Exact area is currently exposed for polynomial Bezier loops using
//! Green's-theorem boundary integrals, the same identities used by
//! [`crate::BezierAreaMoments2`]. That follows Farin's Bernstein polynomial
//! identities in *Curves and Surfaces for CAGD* (5th ed., 2002). Rational conic
//! loops are still concrete boundary loops, but their area returns `None`
//! until rational integral support is added rather than silently sampling.

use hyperreal::Real;

use crate::classify::compare_reals;
use crate::{
    BezierArrangementGraph2, BezierArrangementTraversal2, BezierRetainedLinearOverlapTraversal2,
    BezierSplitFragment2, BezierSubcurve2, Classification, Contour2, ContourPointLocation,
    CurvePolicy, CurveResult, LineSeg2, Point2, Region2, Segment2, UncertaintyReason,
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
/// assignment.  It accepts only materialized polynomial Bezier fragments whose
/// control nets are exact degree-elevated line segments, lowers them to native
/// [`Contour2`] line loops, and then runs the same exact nesting rule used by
/// [`Region2::from_boundary_contours`].  This follows Yap's exact-geometric-
/// computation boundary: unsupported curve families remain explicit evidence
/// gaps rather than being sampled into polygon surrogates.  The containment
/// step uses boundary-first point-in-contour classification as surveyed by
/// Hormann and Agathos, "The Point in Polygon Problem for Arbitrary Polygons,"
/// *Computational Geometry* 20(3), 131-144 (2001).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRetainedLineRegionRoleReport2 {
    roles: Vec<BezierRetainedRegionLoopRole>,
    contours: Vec<Contour2>,
}

impl BezierRetainedLineRegionRoleReport2 {
    /// Constructs a retained line-image role report.
    pub const fn new(roles: Vec<BezierRetainedRegionLoopRole>, contours: Vec<Contour2>) -> Self {
        Self { roles, contours }
    }

    /// Returns one assigned role per retained boundary loop.
    pub fn roles(&self) -> &[BezierRetainedRegionLoopRole] {
        &self.roles
    }

    /// Returns exact native line contours used for role assignment.
    pub fn contours(&self) -> &[Contour2] {
        &self.contours
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

    /// Returns the exact signed area for polynomial-only loops.
    ///
    /// Rational quadratic conics are retained exactly but currently return
    /// `None` here because the rational Green integral is not implemented.
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
        Self { fragments }
    }

    /// Returns retained split fragments in loop order.
    pub fn fragments(&self) -> &[BezierSplitFragment2] {
        &self.fragments
    }

    /// Consumes the loop and returns retained split fragments.
    pub fn into_fragments(self) -> Vec<BezierSplitFragment2> {
        self.fragments
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

    /// Returns exact signed area only for fully native polynomial loops.
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
            }
            loops.push(BezierRetainedBoundaryLoop2::new(fragments));
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
    /// Every retained fragment must be a materialized polynomial Bezier that is
    /// exactly a degree elevation of its endpoint line segment.  The method
    /// lowers those loops to native line contours and assigns even-odd nesting
    /// roles with exact point-in-contour decisions.  It rejects conics,
    /// nonlinear Bezier arcs, algebraic endpoint-image carriers, unresolved
    /// fragments, boundary-touching loops, and uncertain predicate signs.
    pub fn line_image_role_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierRetainedLineRegionRoleReport2>> {
        let mut contours = Vec::with_capacity(self.boundary_loops.len());
        for boundary_loop in &self.boundary_loops {
            let contour = match retained_line_loop_to_contour(boundary_loop, policy)? {
                Classification::Decided(contour) => contour,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            contours.push(contour);
        }

        let roles = match retained_line_loop_roles(&contours, policy)? {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(
            BezierRetainedLineRegionRoleReport2::new(roles, contours),
        ))
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

fn retained_line_loop_to_contour(
    boundary_loop: &BezierRetainedBoundaryLoop2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Contour2>> {
    let mut segments = Vec::with_capacity(boundary_loop.fragments().len());
    for fragment in boundary_loop.fragments() {
        let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let (start, end) = curve.endpoints();
        if !subcurve_is_linearly_parameterized(curve, policy) {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        segments.push(Segment2::Line(LineSeg2::try_new(start, end)?));
    }
    Contour2::try_new(segments).map(Classification::Decided)
}

fn retained_line_loop_roles(
    contours: &[Contour2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<BezierRetainedRegionLoopRole>>> {
    let mut roles = Vec::with_capacity(contours.len());
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
        roles.push(if depth.is_multiple_of(2) {
            BezierRetainedRegionLoopRole::Material
        } else {
            BezierRetainedRegionLoopRole::Hole
        });
    }
    Ok(Classification::Decided(roles))
}

fn subcurve_is_linearly_parameterized(curve: &BezierSubcurve2, policy: &CurvePolicy) -> bool {
    match curve {
        BezierSubcurve2::Quadratic(curve) => {
            point_coordinates_equal(
                curve.control(),
                &linear_control(curve.start(), curve.end(), 1, 2),
                policy,
            ) == Some(true)
        }
        BezierSubcurve2::Cubic(curve) => {
            point_coordinates_equal(
                curve.control1(),
                &linear_control(curve.start(), curve.end(), 1, 3),
                policy,
            ) == Some(true)
                && point_coordinates_equal(
                    curve.control2(),
                    &linear_control(curve.start(), curve.end(), 2, 3),
                    policy,
                ) == Some(true)
        }
        BezierSubcurve2::RationalQuadratic(_) => false,
    }
}

fn linear_control(start: &Point2, end: &Point2, numerator: i32, denominator: i32) -> Point2 {
    let numerator = Real::from(numerator);
    let denominator = Real::from(denominator);
    let complement = &denominator - &numerator;
    Point2::new(
        (((&complement * start.x()) + (&numerator * end.x())) / &denominator)
            .expect("positive integer denominator is nonzero"),
        (((&complement * start.y()) + (&numerator * end.y())) / denominator)
            .expect("positive integer denominator is nonzero"),
    )
}

fn point_coordinates_equal(left: &Point2, right: &Point2, policy: &CurvePolicy) -> Option<bool> {
    Some(
        compare_reals(left.x(), right.x(), policy)? == std::cmp::Ordering::Equal
            && compare_reals(left.y(), right.y(), policy)? == std::cmp::Ordering::Equal,
    )
}

impl BezierSubcurve2 {
    /// Returns exact signed-area contribution when implemented for this curve family.
    pub fn signed_area_contribution(&self) -> CurveResult<Option<Real>> {
        match self {
            Self::Quadratic(curve) => curve.signed_area_contribution().map(Some),
            Self::Cubic(curve) => curve.signed_area_contribution().map(Some),
            Self::RationalQuadratic(_) => Ok(None),
        }
    }
}
