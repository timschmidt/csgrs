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

use crate::{
    BezierArrangementGraph2, BezierArrangementTraversal2, BezierRetainedLinearOverlapTraversal2,
    BezierSplitFragment2, BezierSubcurve2, Classification, CurveResult, UncertaintyReason,
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
