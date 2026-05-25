//! Exact overlap reports for retained Bezier arrangement fragments.
//!
//! Full overlap-aware traversal needs exact artifacts that say which retained
//! fragments share positive-dimensional geometry before a loop walker decides
//! ownership.  This module provides the first such artifact for materialized
//! native Bezier/conic fragments.  It does not sample curves and it does not
//! guess traversal through an overlap.  Instead it replays existing exact
//! curve-relation predicates and emits only certified same-image or line-image
//! overlap pairs.
//!
//! The boundary is intentionally conservative in Yap's exact-geometric-
//! computation sense: see Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 3-23 (1997).  Same polynomial images are
//! certified with Bernstein degree-normalization identities from Farin,
//! *Curves and Surfaces for CAGD* (5th ed., 2002).  Separating overlap
//! reporting from traversal follows the degeneracy discipline emphasized by
//! Foster, Hormann, and Popa, "Clipping simple polygons with degenerate
//! intersections," *Computers & Graphics: X* 2, 100007 (2019): an overlap is a
//! first-class event, not an arbitrary successor choice.

use hyperreal::Real;

use crate::classify::{compare_reals, is_zero};
use crate::{
    BezierArrangementChain2, BezierArrangementGraph2, BezierArrangementTraversal2,
    BezierCurveRelation, BezierParameter2, BezierSplitFragment2, BezierSubcurve2, Classification,
    CurvePolicy, LineLineIntersection, LineSeg2, ParamRange, Point2, UncertaintyReason,
};

/// Exact positive-dimensional overlap relation between two arrangement fragments.
#[derive(Clone, Debug, PartialEq)]
pub enum BezierRetainedOverlapRelation2 {
    /// The fragments have identical control polygons.
    SameControlPolygon,
    /// The fragments have the same polynomial curve image after degree normalization.
    SameCurveImage,
    /// The fragments are line-image Beziers whose supporting finite line segments overlap.
    LineSegmentOverlap {
        /// Exact native line-line overlap result.
        intersection: Box<LineLineIntersection>,
    },
}

/// One certified overlap pair in a retained Bezier arrangement graph.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRetainedOverlap2 {
    first_fragment_index: usize,
    second_fragment_index: usize,
    relation: BezierRetainedOverlapRelation2,
}

impl BezierRetainedOverlap2 {
    /// Constructs a retained overlap pair.
    pub const fn new(
        first_fragment_index: usize,
        second_fragment_index: usize,
        relation: BezierRetainedOverlapRelation2,
    ) -> Self {
        Self {
            first_fragment_index,
            second_fragment_index,
            relation,
        }
    }

    /// Returns the lower graph-fragment index.
    pub const fn first_fragment_index(&self) -> usize {
        self.first_fragment_index
    }

    /// Returns the higher graph-fragment index.
    pub const fn second_fragment_index(&self) -> usize {
        self.second_fragment_index
    }

    /// Returns the certified overlap relation.
    pub const fn relation(&self) -> &BezierRetainedOverlapRelation2 {
        &self.relation
    }
}

/// Exact overlap report for materialized retained Bezier arrangement fragments.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BezierRetainedOverlapReport2 {
    overlaps: Vec<BezierRetainedOverlap2>,
}

/// Retained traversal after consuming certified duplicate materialized overlaps.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRetainedOverlapTraversal2 {
    traversal: BezierArrangementTraversal2,
    overlap_report: BezierRetainedOverlapReport2,
    shadowed_fragment_indices: Vec<usize>,
}

/// Certified extent class for a retained line-image overlap.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierRetainedLineOverlapExtent2 {
    /// The overlap covers both line-image fragments.
    FullBoth,
    /// The overlap covers the first fragment and a strict subrange of the second.
    FullFirstPartialSecond,
    /// The overlap covers a strict subrange of the first and the whole second.
    PartialFirstFullSecond,
    /// The overlap is a strict subrange of both fragments.
    PartialBoth,
}

/// Exact split evidence for a positive-dimensional line-image overlap.
///
/// The stored ranges are the affine parameters of the certified line-segment
/// images, not arbitrary sampled Bezier parameters.  This is the next overlap
/// ownership artifact after pair reporting: future graph splitting can consume
/// the overlap segment endpoints and exact affine ranges while still refusing
/// to conflate them with curve parameters for non-affine line-image Beziers.
/// That distinction is the Yap exact-object boundary in practice; see Yap
/// (1997).  The positive-dimensional segment itself is the ordinary collinear
/// overlap from exact line-line intersection, a standard clipping degeneracy
/// discussed by Foster, Hormann, and Popa (2019).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRetainedLineOverlapSplit2 {
    first_fragment_index: usize,
    second_fragment_index: usize,
    overlap_segment: LineSeg2,
    first_line_range: ParamRange,
    second_line_range: ParamRange,
    extent: BezierRetainedLineOverlapExtent2,
}

/// Exact Bezier-parameter split evidence for a linearly parameterized overlap.
///
/// This is stronger than [`BezierRetainedLineOverlapSplit2`]: the ranges are
/// certified to be valid Bezier parameters, not merely affine coordinates on
/// the endpoint line segment.  The promotion is permitted only for polynomial
/// Bezier control nets that are exact degree elevations of a line segment:
/// quadratic controls `(P0, (P0 + P2)/2, P2)` or cubic controls
/// `(P0, (2P0 + P3)/3, (P0 + 2P3)/3, P3)`.  These are the standard Bernstein
/// degree-elevation identities in Farin (2002).  Per Yap (1997), general
/// collinear-but-nonlinear line images stay exact line-image evidence until a
/// separate inverse-parameter construction exists.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRetainedLinearOverlapSplit2 {
    first_fragment_index: usize,
    second_fragment_index: usize,
    overlap_segment: LineSeg2,
    first_bezier_range: ParamRange,
    second_bezier_range: ParamRange,
    extent: BezierRetainedLineOverlapExtent2,
}

/// One fragment in a graph refined by certified linear-overlap split points.
///
/// The `local_range` is measured in the original retained graph fragment's
/// Bezier parameter, not in the source curve's global parameter.  That makes
/// the provenance replayable even after repeated graph refinements: each
/// refined carrier says exactly which original fragment and exact local
/// interval produced it.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRetainedOverlapRefinedFragment2 {
    original_fragment_index: usize,
    local_range: ParamRange,
}

/// Graph refined at all certified linearly-parameterized overlap endpoints.
///
/// This is an ownership-preparation artifact.  It does not decide which side
/// owns an overlap and it does not traverse through positive-dimensional
/// degeneracies.  It only turns exact split evidence into a new retained graph
/// whose fragment boundaries include the overlap endpoints.  Per Yap (1997),
/// that keeps the constructed subcurves exact and report-bearing before any
/// topology consumer chooses ownership.  The subcurves are materialized by de
/// Casteljau subdivision, de Casteljau (1959), using the Bernstein identities
/// summarized by Farin (2002).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRetainedLinearOverlapSplitGraph2 {
    graph: BezierArrangementGraph2,
    refined_fragments: Vec<BezierRetainedOverlapRefinedFragment2>,
    overlap_report: BezierRetainedOverlapReport2,
    split_plan: Vec<BezierRetainedLinearOverlapSplit2>,
}

impl BezierRetainedLinearOverlapSplit2 {
    /// Constructs exact Bezier-parameter split evidence for a linear overlap.
    pub const fn new(
        first_fragment_index: usize,
        second_fragment_index: usize,
        overlap_segment: LineSeg2,
        first_bezier_range: ParamRange,
        second_bezier_range: ParamRange,
        extent: BezierRetainedLineOverlapExtent2,
    ) -> Self {
        Self {
            first_fragment_index,
            second_fragment_index,
            overlap_segment,
            first_bezier_range,
            second_bezier_range,
            extent,
        }
    }

    /// Returns the lower graph-fragment index.
    pub const fn first_fragment_index(&self) -> usize {
        self.first_fragment_index
    }

    /// Returns the higher graph-fragment index.
    pub const fn second_fragment_index(&self) -> usize {
        self.second_fragment_index
    }

    /// Returns the exact overlap segment.
    pub const fn overlap_segment(&self) -> &LineSeg2 {
        &self.overlap_segment
    }

    /// Returns the exact Bezier parameter range on the first fragment.
    pub const fn first_bezier_range(&self) -> &ParamRange {
        &self.first_bezier_range
    }

    /// Returns the exact Bezier parameter range on the second fragment.
    pub const fn second_bezier_range(&self) -> &ParamRange {
        &self.second_bezier_range
    }

    /// Returns whether the overlap is full or partial on each side.
    pub const fn extent(&self) -> BezierRetainedLineOverlapExtent2 {
        self.extent
    }
}

impl BezierRetainedOverlapRefinedFragment2 {
    /// Constructs provenance for one refined overlap-split graph fragment.
    pub const fn new(original_fragment_index: usize, local_range: ParamRange) -> Self {
        Self {
            original_fragment_index,
            local_range,
        }
    }

    /// Returns the fragment index in the graph that was refined.
    pub const fn original_fragment_index(&self) -> usize {
        self.original_fragment_index
    }

    /// Returns the exact local range in the original retained fragment.
    pub const fn local_range(&self) -> &ParamRange {
        &self.local_range
    }
}

impl BezierRetainedLinearOverlapSplitGraph2 {
    /// Constructs a retained graph refinement and its replay metadata.
    pub const fn new(
        graph: BezierArrangementGraph2,
        refined_fragments: Vec<BezierRetainedOverlapRefinedFragment2>,
        overlap_report: BezierRetainedOverlapReport2,
        split_plan: Vec<BezierRetainedLinearOverlapSplit2>,
    ) -> Self {
        Self {
            graph,
            refined_fragments,
            overlap_report,
            split_plan,
        }
    }

    /// Returns the refined retained arrangement graph.
    pub const fn graph(&self) -> &BezierArrangementGraph2 {
        &self.graph
    }

    /// Returns provenance for every fragment in [`Self::graph`].
    pub fn refined_fragments(&self) -> &[BezierRetainedOverlapRefinedFragment2] {
        &self.refined_fragments
    }

    /// Returns the overlap report consumed to build the split plan.
    pub const fn overlap_report(&self) -> &BezierRetainedOverlapReport2 {
        &self.overlap_report
    }

    /// Returns the certified linear-overlap splits used for refinement.
    pub fn split_plan(&self) -> &[BezierRetainedLinearOverlapSplit2] {
        &self.split_plan
    }

    /// Consumes this refinement and returns all parts.
    pub fn into_parts(
        self,
    ) -> (
        BezierArrangementGraph2,
        Vec<BezierRetainedOverlapRefinedFragment2>,
        BezierRetainedOverlapReport2,
        Vec<BezierRetainedLinearOverlapSplit2>,
    ) {
        (
            self.graph,
            self.refined_fragments,
            self.overlap_report,
            self.split_plan,
        )
    }
}

impl BezierRetainedLineOverlapSplit2 {
    /// Constructs exact line-image split evidence.
    pub const fn new(
        first_fragment_index: usize,
        second_fragment_index: usize,
        overlap_segment: LineSeg2,
        first_line_range: ParamRange,
        second_line_range: ParamRange,
        extent: BezierRetainedLineOverlapExtent2,
    ) -> Self {
        Self {
            first_fragment_index,
            second_fragment_index,
            overlap_segment,
            first_line_range,
            second_line_range,
            extent,
        }
    }

    /// Returns the lower graph-fragment index.
    pub const fn first_fragment_index(&self) -> usize {
        self.first_fragment_index
    }

    /// Returns the higher graph-fragment index.
    pub const fn second_fragment_index(&self) -> usize {
        self.second_fragment_index
    }

    /// Returns the exact overlap segment.
    pub const fn overlap_segment(&self) -> &LineSeg2 {
        &self.overlap_segment
    }

    /// Returns the affine line range on the first fragment image.
    pub const fn first_line_range(&self) -> &ParamRange {
        &self.first_line_range
    }

    /// Returns the affine line range on the second fragment image.
    pub const fn second_line_range(&self) -> &ParamRange {
        &self.second_line_range
    }

    /// Returns whether the overlap is full or partial on each side.
    pub const fn extent(&self) -> BezierRetainedLineOverlapExtent2 {
        self.extent
    }
}

impl BezierArrangementGraph2 {
    /// Traverses retained fragments after deduplicating exact duplicate overlaps.
    ///
    /// This is the first overlap-consuming traversal stage.  It accepts only
    /// overlaps whose fragment images and oriented endpoints are certified
    /// equal, shadows the duplicate fragment, and then replays retained tangent
    /// traversal on the remaining graph.  Partial line overlaps and reversed
    /// same-image overlaps are still boundary uncertainty because consuming
    /// them requires ownership and splitting rules not represented by this
    /// slice.
    pub fn traverse_retained_deduplicating_materialized_overlaps(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierRetainedOverlapTraversal2> {
        let overlap_report = match BezierRetainedOverlapReport2::from_graph(self, policy) {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let shadowed_fragment_indices =
            match duplicate_shadow_indices(self, &overlap_report, policy) {
                Classification::Decided(indices) => indices,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };

        let traversal = if shadowed_fragment_indices.is_empty() {
            match self.traverse_retained_with_tangent_order(policy) {
                Classification::Decided(traversal) => traversal,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        } else {
            let (filtered, original_indices) = filtered_graph(self, &shadowed_fragment_indices);
            if filtered.is_empty() {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            let filtered_traversal = match filtered.traverse_retained_with_tangent_order(policy) {
                Classification::Decided(traversal) => traversal,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
            remap_traversal_indices(filtered_traversal, &original_indices)
        };

        Classification::Decided(BezierRetainedOverlapTraversal2 {
            traversal,
            overlap_report,
            shadowed_fragment_indices,
        })
    }

    /// Splits retained materialized fragments at certified linear-overlap endpoints.
    ///
    /// The method is deliberately narrower than a full overlap walker: it
    /// requires all line-image overlaps in the report to have exact Bezier
    /// parameter ranges from [`BezierRetainedOverlapReport2::linear_bezier_overlap_splits`].
    /// It then inserts the range endpoints into the affected fragments and
    /// materializes exact subcurves with de Casteljau subdivision.  Same-image
    /// duplicate overlaps are reported but do not add boundaries; nonlinear
    /// line images, unresolved endpoint carriers, or uncertain ordering remain
    /// explicit uncertainty.
    pub fn split_retained_linear_overlaps(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierRetainedLinearOverlapSplitGraph2> {
        let overlap_report = match BezierRetainedOverlapReport2::from_graph(self, policy) {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let split_plan = match overlap_report.linear_bezier_overlap_splits(self, policy) {
            Classification::Decided(split_plan) => split_plan,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let boundaries = match linear_overlap_boundaries(self.len(), &split_plan, policy) {
            Classification::Decided(boundaries) => boundaries,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        let (graph, refined_fragments) = match refine_graph_at_boundaries(self, &boundaries, policy)
        {
            Classification::Decided(refinement) => refinement,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        Classification::Decided(BezierRetainedLinearOverlapSplitGraph2::new(
            graph,
            refined_fragments,
            overlap_report,
            split_plan,
        ))
    }
}

impl BezierRetainedOverlapTraversal2 {
    /// Returns the traversal over original graph-fragment indices.
    pub const fn traversal(&self) -> &BezierArrangementTraversal2 {
        &self.traversal
    }

    /// Returns the exact overlap report consumed by this traversal stage.
    pub const fn overlap_report(&self) -> &BezierRetainedOverlapReport2 {
        &self.overlap_report
    }

    /// Returns original graph-fragment indices shadowed as exact duplicates.
    pub fn shadowed_fragment_indices(&self) -> &[usize] {
        &self.shadowed_fragment_indices
    }

    /// Consumes the report and returns its parts.
    pub fn into_parts(
        self,
    ) -> (
        BezierArrangementTraversal2,
        BezierRetainedOverlapReport2,
        Vec<usize>,
    ) {
        (
            self.traversal,
            self.overlap_report,
            self.shadowed_fragment_indices,
        )
    }
}

impl BezierRetainedOverlapReport2 {
    /// Scans a retained arrangement graph for certified materialized overlaps.
    ///
    /// Algebraic endpoint-image and unresolved fragments are not overlap-
    /// resolved here because endpoint evidence alone is not a curve-image
    /// overlap proof.  Materialized pairs whose relation remains unresolved
    /// return boundary uncertainty so callers cannot treat an incomplete scan
    /// as a no-overlap proof.
    pub fn from_graph(
        graph: &BezierArrangementGraph2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut overlaps = Vec::new();
        for first_index in 0..graph.fragments().len() {
            for second_index in (first_index + 1)..graph.fragments().len() {
                let relation = match materialized_overlap_relation(
                    graph.fragments()[first_index].fragment(),
                    graph.fragments()[second_index].fragment(),
                    policy,
                ) {
                    Classification::Decided(relation) => relation,
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                };
                if let Some(relation) = relation {
                    overlaps.push(BezierRetainedOverlap2::new(
                        first_index,
                        second_index,
                        relation,
                    ));
                }
            }
        }
        Classification::Decided(Self { overlaps })
    }

    /// Constructs a report from already-certified overlaps.
    pub const fn new(overlaps: Vec<BezierRetainedOverlap2>) -> Self {
        Self { overlaps }
    }

    /// Returns certified overlap pairs.
    pub fn overlaps(&self) -> &[BezierRetainedOverlap2] {
        &self.overlaps
    }

    /// Consumes the report and returns certified overlap pairs.
    pub fn into_overlaps(self) -> Vec<BezierRetainedOverlap2> {
        self.overlaps
    }

    /// Returns true when the scan found no certified materialized overlaps.
    pub fn is_empty(&self) -> bool {
        self.overlaps.is_empty()
    }

    /// Returns the number of certified materialized overlap pairs.
    pub fn len(&self) -> usize {
        self.overlaps.len()
    }

    /// Extracts exact line-image overlap split evidence from this report.
    ///
    /// Same-control and same-curve-image overlaps are full curve-image
    /// degeneracies and do not have line affine ranges here.  Only
    /// [`BezierRetainedOverlapRelation2::LineSegmentOverlap`] contributes.
    pub fn line_overlap_splits(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<Vec<BezierRetainedLineOverlapSplit2>> {
        let mut splits = Vec::new();
        for overlap in &self.overlaps {
            let BezierRetainedOverlapRelation2::LineSegmentOverlap { intersection } =
                overlap.relation()
            else {
                continue;
            };
            let LineLineIntersection::Overlap {
                segment,
                a_range,
                b_range,
            } = intersection.as_ref()
            else {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            };
            let extent = match line_overlap_extent(a_range, b_range, policy) {
                Some(extent) => extent,
                None => return Classification::Uncertain(UncertaintyReason::Ordering),
            };
            splits.push(BezierRetainedLineOverlapSplit2::new(
                overlap.first_fragment_index(),
                overlap.second_fragment_index(),
                segment.clone(),
                a_range.clone(),
                b_range.clone(),
                extent,
            ));
        }
        Classification::Decided(splits)
    }

    /// Promotes exact line-image overlaps to Bezier-parameter split evidence.
    ///
    /// This succeeds only when every line-image overlap in the report is backed
    /// by materialized polynomial Bezier fragments with certified linear
    /// parameterization.  A single nonlinear line image makes the result
    /// unsupported rather than partially emitted, because callers use this
    /// method as a complete graph-splitting precondition.
    pub fn linear_bezier_overlap_splits(
        &self,
        graph: &BezierArrangementGraph2,
        policy: &CurvePolicy,
    ) -> Classification<Vec<BezierRetainedLinearOverlapSplit2>> {
        let line_splits = match self.line_overlap_splits(policy) {
            Classification::Decided(splits) => splits,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let mut promoted = Vec::new();
        for split in line_splits {
            if !fragment_is_linearly_parameterized(graph, split.first_fragment_index(), policy)
                || !fragment_is_linearly_parameterized(graph, split.second_fragment_index(), policy)
            {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            }
            promoted.push(BezierRetainedLinearOverlapSplit2::new(
                split.first_fragment_index(),
                split.second_fragment_index(),
                split.overlap_segment().clone(),
                split.first_line_range().clone(),
                split.second_line_range().clone(),
                split.extent(),
            ));
        }
        Classification::Decided(promoted)
    }
}

fn fragment_is_linearly_parameterized(
    graph: &BezierArrangementGraph2,
    fragment_index: usize,
    policy: &CurvePolicy,
) -> bool {
    let Some(fragment) = graph.fragments().get(fragment_index) else {
        return false;
    };
    let BezierSplitFragment2::Materialized { curve, .. } = fragment.fragment() else {
        return false;
    };
    match curve {
        BezierSubcurve2::Quadratic(curve) => {
            point_coordinates_equal(
                curve.control(),
                &midpoint(curve.start(), curve.end()),
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

fn midpoint(start: &Point2, end: &Point2) -> Point2 {
    linear_control(start, end, 1, 2)
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

fn line_overlap_extent(
    first: &ParamRange,
    second: &ParamRange,
    policy: &CurvePolicy,
) -> Option<BezierRetainedLineOverlapExtent2> {
    let first_full = unit_range(first, policy)?;
    let second_full = unit_range(second, policy)?;
    Some(match (first_full, second_full) {
        (true, true) => BezierRetainedLineOverlapExtent2::FullBoth,
        (true, false) => BezierRetainedLineOverlapExtent2::FullFirstPartialSecond,
        (false, true) => BezierRetainedLineOverlapExtent2::PartialFirstFullSecond,
        (false, false) => BezierRetainedLineOverlapExtent2::PartialBoth,
    })
}

fn linear_overlap_boundaries(
    fragment_count: usize,
    split_plan: &[BezierRetainedLinearOverlapSplit2],
    policy: &CurvePolicy,
) -> Classification<Vec<Vec<Real>>> {
    let mut boundaries = vec![vec![Real::zero(), Real::one()]; fragment_count];
    for split in split_plan {
        if !push_boundary(
            &mut boundaries,
            split.first_fragment_index(),
            split.first_bezier_range().start().clone(),
            policy,
        ) || !push_boundary(
            &mut boundaries,
            split.first_fragment_index(),
            split.first_bezier_range().end().clone(),
            policy,
        ) || !push_boundary(
            &mut boundaries,
            split.second_fragment_index(),
            split.second_bezier_range().start().clone(),
            policy,
        ) || !push_boundary(
            &mut boundaries,
            split.second_fragment_index(),
            split.second_bezier_range().end().clone(),
            policy,
        ) {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
    }

    for fragment_boundaries in &mut boundaries {
        match sort_and_dedup_boundaries(fragment_boundaries, policy) {
            Some(()) => {}
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(boundaries)
}

fn push_boundary(
    boundaries: &mut [Vec<Real>],
    fragment_index: usize,
    boundary: Real,
    _policy: &CurvePolicy,
) -> bool {
    let Some(fragment_boundaries) = boundaries.get_mut(fragment_index) else {
        return false;
    };
    fragment_boundaries.push(boundary);
    true
}

fn sort_and_dedup_boundaries(boundaries: &mut Vec<Real>, policy: &CurvePolicy) -> Option<()> {
    for index in 1..boundaries.len() {
        let mut cursor = index;
        while cursor > 0 {
            match compare_reals(&boundaries[cursor], &boundaries[cursor - 1], policy)? {
                std::cmp::Ordering::Less => {
                    boundaries.swap(cursor, cursor - 1);
                    cursor -= 1;
                }
                std::cmp::Ordering::Equal | std::cmp::Ordering::Greater => break,
            }
        }
    }

    let mut deduped = Vec::with_capacity(boundaries.len());
    for boundary in boundaries.drain(..) {
        if deduped.last().is_some_and(|last| {
            compare_reals(last, &boundary, policy) == Some(std::cmp::Ordering::Equal)
        }) {
            continue;
        }
        deduped.push(boundary);
    }
    *boundaries = deduped;
    Some(())
}

fn refine_graph_at_boundaries(
    graph: &BezierArrangementGraph2,
    boundaries: &[Vec<Real>],
    policy: &CurvePolicy,
) -> Classification<(
    BezierArrangementGraph2,
    Vec<BezierRetainedOverlapRefinedFragment2>,
)> {
    let mut refined_graph_fragments = Vec::new();
    let mut refined_fragments = Vec::new();

    for (original_index, arrangement_fragment) in graph.fragments().iter().enumerate() {
        let Some(fragment_boundaries) = boundaries.get(original_index) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        if fragment_boundaries.len() <= 2 {
            refined_graph_fragments.push(arrangement_fragment.clone());
            refined_fragments.push(BezierRetainedOverlapRefinedFragment2::new(
                original_index,
                ParamRange::new(Real::zero(), Real::one()),
            ));
            continue;
        }

        let BezierSplitFragment2::Materialized { start, end, curve } =
            arrangement_fragment.fragment()
        else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        let (Some(source_start), Some(source_end)) = (start.as_exact(), end.as_exact()) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };

        for pair in fragment_boundaries.windows(2) {
            let local_start = pair[0].clone();
            let local_end = pair[1].clone();
            if compare_reals(&local_start, &local_end, policy) == Some(std::cmp::Ordering::Equal) {
                continue;
            }
            let subcurve = match subcurve_between_local(curve, &local_start, &local_end, policy) {
                Classification::Decided(subcurve) => subcurve,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
            let refined_start = compose_source_parameter(source_start, source_end, &local_start);
            let refined_end = compose_source_parameter(source_start, source_end, &local_end);
            let local_range = ParamRange::new(local_start, local_end);
            refined_graph_fragments.push(crate::BezierArrangementFragment2::new(
                arrangement_fragment.source_curve_index(),
                arrangement_fragment.source_fragment_index(),
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(refined_start),
                    end: BezierParameter2::Exact(refined_end),
                    curve: subcurve,
                },
            ));
            refined_fragments.push(BezierRetainedOverlapRefinedFragment2::new(
                original_index,
                local_range,
            ));
        }
    }

    Classification::Decided((
        BezierArrangementGraph2::new(refined_graph_fragments),
        refined_fragments,
    ))
}

fn subcurve_between_local(
    curve: &BezierSubcurve2,
    start: &Real,
    end: &Real,
    policy: &CurvePolicy,
) -> Classification<BezierSubcurve2> {
    let result = match curve {
        BezierSubcurve2::Quadratic(curve) => curve
            .subcurve_between_exact(start, end, policy)
            .map(BezierSubcurve2::Quadratic),
        BezierSubcurve2::Cubic(curve) => curve
            .subcurve_between_exact(start, end, policy)
            .map(BezierSubcurve2::Cubic),
        BezierSubcurve2::RationalQuadratic(curve) => curve
            .subcurve_between_exact(start, end, policy)
            .map(BezierSubcurve2::RationalQuadratic),
    };
    match result {
        Ok(curve) => Classification::Decided(curve),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn compose_source_parameter(source_start: &Real, source_end: &Real, local: &Real) -> Real {
    source_start + (&(source_end - source_start) * local)
}

fn unit_range(range: &ParamRange, policy: &CurvePolicy) -> Option<bool> {
    Some(
        crate::classify::compare_reals(range.start(), &hyperreal::Real::zero(), policy)?
            == std::cmp::Ordering::Equal
            && crate::classify::compare_reals(range.end(), &hyperreal::Real::one(), policy)?
                == std::cmp::Ordering::Equal,
    )
}

fn duplicate_shadow_indices(
    graph: &BezierArrangementGraph2,
    report: &BezierRetainedOverlapReport2,
    policy: &CurvePolicy,
) -> Classification<Vec<usize>> {
    let mut shadowed = vec![false; graph.len()];
    for overlap in report.overlaps() {
        if !overlap_relation_can_shadow_duplicate(overlap.relation()) {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        let same_orientation = match oriented_materialized_endpoints_equal(
            graph,
            overlap.first_fragment_index(),
            overlap.second_fragment_index(),
            policy,
        ) {
            Some(value) => value,
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        };
        if !same_orientation {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        if !shadowed[overlap.first_fragment_index()] {
            shadowed[overlap.second_fragment_index()] = true;
        }
    }

    Classification::Decided(
        shadowed
            .into_iter()
            .enumerate()
            .filter_map(|(index, shadowed)| shadowed.then_some(index))
            .collect(),
    )
}

fn overlap_relation_can_shadow_duplicate(relation: &BezierRetainedOverlapRelation2) -> bool {
    matches!(
        relation,
        BezierRetainedOverlapRelation2::SameControlPolygon
            | BezierRetainedOverlapRelation2::SameCurveImage
            | BezierRetainedOverlapRelation2::LineSegmentOverlap { .. }
    )
}

fn oriented_materialized_endpoints_equal(
    graph: &BezierArrangementGraph2,
    first_index: usize,
    second_index: usize,
    policy: &CurvePolicy,
) -> Option<bool> {
    let first = materialized_endpoints(graph.fragments().get(first_index)?.fragment())?;
    let second = materialized_endpoints(graph.fragments().get(second_index)?.fragment())?;
    Some(points_equal(&first.0, &second.0, policy)? && points_equal(&first.1, &second.1, policy)?)
}

fn materialized_endpoints(fragment: &BezierSplitFragment2) -> Option<(Point2, Point2)> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => Some(curve.endpoints()),
        BezierSplitFragment2::AlgebraicEndpointImages { .. }
        | BezierSplitFragment2::Unresolved { .. } => None,
    }
}

fn points_equal(left: &Point2, right: &Point2, policy: &CurvePolicy) -> Option<bool> {
    is_zero(&left.distance_squared(right), policy)
}

fn filtered_graph(
    graph: &BezierArrangementGraph2,
    shadowed_fragment_indices: &[usize],
) -> (BezierArrangementGraph2, Vec<usize>) {
    let mut original_indices = Vec::new();
    let mut fragments = Vec::new();
    for (index, fragment) in graph.fragments().iter().enumerate() {
        if shadowed_fragment_indices.binary_search(&index).is_ok() {
            continue;
        }
        original_indices.push(index);
        fragments.push(fragment.clone());
    }
    (BezierArrangementGraph2::new(fragments), original_indices)
}

fn remap_traversal_indices(
    traversal: BezierArrangementTraversal2,
    original_indices: &[usize],
) -> BezierArrangementTraversal2 {
    BezierArrangementTraversal2::new(
        traversal
            .into_chains()
            .into_iter()
            .map(|chain| {
                let closed = chain.is_closed();
                let indices = chain
                    .into_fragment_indices()
                    .into_iter()
                    .map(|index| original_indices[index])
                    .collect();
                BezierArrangementChain2::new(indices, closed)
            })
            .collect(),
    )
}

fn materialized_overlap_relation(
    first: &BezierSplitFragment2,
    second: &BezierSplitFragment2,
    policy: &CurvePolicy,
) -> Classification<Option<BezierRetainedOverlapRelation2>> {
    let (
        BezierSplitFragment2::Materialized { curve: first, .. },
        BezierSplitFragment2::Materialized { curve: second, .. },
    ) = (first, second)
    else {
        return Classification::Decided(None);
    };

    match subcurve_relation(first, second, policy) {
        Classification::Decided(BezierCurveRelation::SameControlPolygon) => {
            Classification::Decided(Some(BezierRetainedOverlapRelation2::SameControlPolygon))
        }
        Classification::Decided(BezierCurveRelation::SameCurveImage) => {
            Classification::Decided(Some(BezierRetainedOverlapRelation2::SameCurveImage))
        }
        Classification::Decided(BezierCurveRelation::LineSegmentIntersection { intersection }) => {
            match intersection {
                LineLineIntersection::Overlap { .. } => Classification::Decided(Some(
                    BezierRetainedOverlapRelation2::LineSegmentOverlap {
                        intersection: Box::new(intersection),
                    },
                )),
                LineLineIntersection::Point { .. } | LineLineIntersection::None => {
                    Classification::Decided(None)
                }
                LineLineIntersection::Uncertain { reason } => Classification::Uncertain(reason),
            }
        }
        Classification::Decided(
            BezierCurveRelation::BoundingBoxesDisjoint
            | BezierCurveRelation::NoIntersection
            | BezierCurveRelation::SharedEndpoint
            | BezierCurveRelation::IntersectionPoints { .. }
            | BezierCurveRelation::EndpointIntersections { .. }
            | BezierCurveRelation::IntersectionRegions { .. },
        ) => Classification::Decided(None),
        Classification::Decided(BezierCurveRelation::Unresolved) => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn subcurve_relation(
    first: &BezierSubcurve2,
    second: &BezierSubcurve2,
    policy: &CurvePolicy,
) -> Classification<BezierCurveRelation> {
    match (first, second) {
        (BezierSubcurve2::Quadratic(first), BezierSubcurve2::Quadratic(second)) => {
            first.relation_to_quadratic(second, policy)
        }
        (BezierSubcurve2::Quadratic(first), BezierSubcurve2::Cubic(second)) => {
            first.relation_to_cubic(second, policy)
        }
        (BezierSubcurve2::Cubic(first), BezierSubcurve2::Quadratic(second)) => {
            first.relation_to_quadratic(second, policy)
        }
        (BezierSubcurve2::Cubic(first), BezierSubcurve2::Cubic(second)) => {
            first.relation_to_cubic(second, policy)
        }
        (BezierSubcurve2::RationalQuadratic(first), BezierSubcurve2::RationalQuadratic(second)) => {
            first.relation_to_rational_quadratic(second, policy)
        }
        (BezierSubcurve2::RationalQuadratic(first), BezierSubcurve2::Quadratic(second)) => {
            first.relation_to_quadratic(second, policy)
        }
        (BezierSubcurve2::Quadratic(first), BezierSubcurve2::RationalQuadratic(second)) => {
            first.relation_to_rational_quadratic(second, policy)
        }
        (BezierSubcurve2::RationalQuadratic(first), BezierSubcurve2::Cubic(second)) => {
            first.relation_to_cubic(second, policy)
        }
        (BezierSubcurve2::Cubic(first), BezierSubcurve2::RationalQuadratic(second)) => {
            first.relation_to_rational_quadratic(second, policy)
        }
    }
}
