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

use crate::{
    BezierArrangementGraph2, BezierCurveRelation, BezierSplitFragment2, BezierSubcurve2,
    Classification, CurvePolicy, LineLineIntersection, UncertaintyReason,
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
