//! Curve primitives built on the hyper geometry stack.
//!
//! This crate starts with a line/circular-arc core using [`hyperreal::Real`]
//! coordinates and the `hyperlimit` predicate policy model. The core topology
//! follows the robust-computation principle of deciding predicates before branching; see
//! Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast Robust
//! Geometric Predicates" (*Discrete & Computational Geometry* 18(3), 305-363,
//! 1997).

mod area;
mod bbox;
mod bezier;
mod bezier_boolean;
mod bezier_fit;
mod bezier_fit_source;
mod bezier_flatten;
mod bezier_metric;
mod bezier_moment;
mod bezier_offset;
mod bezier_region;
mod bezier_topology;
mod boolean;
mod boolean_boundary;
mod boolean_report;
mod boolean_traversal_report;
mod bulge;
mod classify;
mod contour;
mod curve_string;
mod error;
mod events;
mod facts;
mod finite_projection;
mod fragment;
mod intersect;
mod offset;
mod point;
mod policy;
mod prepared;
mod prepared_boolean;
mod rational_bezier;
mod reconstruct;
mod region;
mod region_boolean;
mod region_events;
mod region_fragments;
mod region_nesting;
mod segment;
mod self_intersect;
mod split;
mod transform;
#[cfg(feature = "triangulation")]
mod triangulation;

pub use area::{
    ContourAreaUnsupportedReason, ContourAreaUnsupportedSegment2, ContourSignedAreaReport2,
    RegionAreaContourRole, RegionAreaUnsupportedContour2, RegionFilledAreaReport2,
};
pub use bbox::Aabb2;
pub use bezier::{BezierEndpoint, CubicBezier2, EndpointTangent2, QuadraticBezier2};
pub use bezier_boolean::{
    BezierBooleanArrangementReadinessReport2, BezierBooleanArrangementReadinessStatus,
    BezierBooleanAssemblyReadinessReport2, BezierBooleanAssemblyReadinessStatus,
    BezierBooleanBatchHandoffReport2, BezierBooleanBatchHandoffStatus,
    BezierBooleanConstructionReadinessReport2, BezierBooleanConstructionReadinessStatus,
    BezierBooleanCubicFragmentReport2, BezierBooleanEmissionPlanReport2,
    BezierBooleanEmissionPlanStatus, BezierBooleanFragmentConstructionStatus,
    BezierBooleanFragmentOwnershipLocation, BezierBooleanHandoffReport2,
    BezierBooleanHandoffStatus, BezierBooleanLoopAssemblyPlanReport2,
    BezierBooleanLoopAssemblyPlanStatus, BezierBooleanLoopClosureReport2,
    BezierBooleanLoopClosureStatus, BezierBooleanLoopContainmentFact2,
    BezierBooleanLoopContainmentFactReport2, BezierBooleanLoopContainmentFactStatus,
    BezierBooleanLoopGraphFactReport2, BezierBooleanLoopGraphFactStatus,
    BezierBooleanLoopGraphFacts2, BezierBooleanLoopGraphTraversalReport2,
    BezierBooleanLoopGraphTraversalStatus, BezierBooleanLoopGraphWalkReport2,
    BezierBooleanLoopGraphWalkStatus, BezierBooleanLoopNestingDepthFact2,
    BezierBooleanLoopNestingDepthFactReport2, BezierBooleanLoopNestingDepthFactStatus,
    BezierBooleanLoopNestingRoleReport2, BezierBooleanLoopNestingRoleStatus,
    BezierBooleanLoopRoleAssignmentReport2, BezierBooleanLoopRoleAssignmentStatus,
    BezierBooleanOperandOwnershipLocationReport2, BezierBooleanOperandOwnershipLocationStatus,
    BezierBooleanOutputLoop2, BezierBooleanOutputLoopReport2, BezierBooleanOutputLoopRole,
    BezierBooleanOutputLoopStatus, BezierBooleanOverlapEvent2,
    BezierBooleanOverlapResolutionReport2, BezierBooleanOverlapResolutionStatus,
    BezierBooleanOwnedTraversalStep2, BezierBooleanOwnershipClassificationReport2,
    BezierBooleanOwnershipClassificationStatus, BezierBooleanOwnershipFact2,
    BezierBooleanOwnershipFactReport2, BezierBooleanOwnershipFactStatus,
    BezierBooleanPathSchedulerReport2, BezierBooleanPathSchedulerStatus, BezierBooleanPointEvent2,
    BezierBooleanQuadraticFragmentReport2, BezierBooleanRationalQuadraticFragmentReport2,
    BezierBooleanRegionAssemblyReport2, BezierBooleanRegionAssemblyStatus,
    BezierBooleanResolvedOverlapEvent2, BezierBooleanResultReport2, BezierBooleanResultStatus,
    BezierBooleanSplitInsertionReport2, BezierBooleanSplitInsertionStatus,
    BezierBooleanSplitParameterLocation, BezierBooleanSplitPlanAuditReport2,
    BezierBooleanSplitPlanAuditStatus, BezierBooleanSplitPlanReport2, BezierBooleanSplitPlanStatus,
    BezierBooleanTraversalOperand, BezierBooleanTraversalPreconditionReport2,
    BezierBooleanTraversalPreconditionStatus, BezierBooleanTraversalScheduleReport2,
    BezierBooleanTraversalScheduleStatus, BezierBooleanTraversalStep2,
    BezierBooleanUniformOwnershipFactReport2, BezierBooleanUniformOwnershipFactStatus,
    BezierPathRangeBatchReport2, BezierPathRangeBatchStatus, BezierPathRangeOrderReport2,
    BezierPathRangeOrderStatus,
};
pub use bezier_fit::{
    BezierFitBoundKind, BezierFitCertificate, BezierFitErrorMetric, BezierFitReadinessReport2,
    BezierFitReadinessStatus, BezierLineFitRelation, BezierLineImageFitRelation,
    BezierPointFitRelation, BezierPointImageFitRelation, CertifiedBezierLineFit2,
    CertifiedBezierLineImage2, CertifiedBezierLineImageOffset2, CertifiedBezierLineOffset2,
    CertifiedBezierPointFit2, CertifiedBezierPointImage2,
};
pub use bezier_fit_source::{
    BezierFitSourceBatchReport2, BezierFitSourcePrefixSums2, BezierFitSourceReport2,
};
pub use bezier_flatten::{
    BezierFlatteningCertificate, BezierFlatteningOptions, BezierSimplificationBoundKind,
    BezierSimplificationCertificate, BezierSimplificationErrorMetric, CertifiedBezierPolyline2,
    CertifiedBezierPolylineOffset2, DisplayBezierOffsetPolyline2,
};
pub use bezier_metric::{BezierArcLengthParameterRegion2, BezierLengthBounds2};
pub use bezier_moment::{BezierAreaMomentPrefixSums2, BezierAreaMoments2, BezierAreaPrefixSums2};
pub use bezier_offset::{
    BezierOffsetAdapterReport2, BezierOffsetAdapterStatus, BezierOffsetCandidate2,
    BezierOffsetPreflight2, BezierOffsetRisk,
};
pub use bezier_region::{
    BezierIntersectionRegionFacts, BezierIntersectionRegionIsolationBudget,
    BezierIntersectionRegionIsolationCertificate, BezierIntersectionRegionIsolationReport,
    BezierIntersectionRegionIsolationStopReason, BezierIntersectionRegionRefinement,
    BezierIntersectionRegionRefinementAction, BezierIntersectionRegionShape,
    BezierIntersectionRegionSummary, BezierRegionWidthStatus, bezier_intersection_region_facts,
    certify_bezier_intersection_region_isolation, isolate_bezier_intersection_regions,
    isolate_bezier_intersection_regions_until_width, refine_bezier_intersection_region,
    refine_bezier_intersection_regions, summarize_bezier_intersection_regions,
};
pub use bezier_topology::{
    Axis2, BezierCurveIntersectionPoint, BezierCurveIntersectionRegion, BezierCurveRelation,
    BezierCuspClassification, BezierGraphContact, BezierInflectionClassification,
    BezierLineContact, BezierLineContactKind, BezierLineContactRelation, BezierLineRelation,
    BezierMonotoneGraphContactOrder, BezierMonotoneGraphOrder, BezierMonotoneSpan,
};
pub use boolean::{
    BooleanFragmentAction, BooleanFragmentClassification, BooleanFragmentSelection, BooleanOp,
};
pub use boolean_boundary::{
    BooleanBoundaryChain, BooleanBoundaryChainSet, BooleanBoundaryFragmentSet, BooleanBoundaryLoop,
    BooleanBoundaryLoopSet, DirectedBooleanFragment,
};
pub use boolean_report::{
    BooleanBoundaryAuditStatus, BooleanBoundaryContourAuditReport2, BooleanBoundaryContourReport2,
    BooleanBoundaryLoopAuditReport2, BooleanBoundaryLoopReport2, BooleanRegionAuditReport2,
    BooleanRegionAuditStatus, BooleanRegionPipelineReport2, BooleanRegionReport2,
};
pub use boolean_traversal_report::{
    BooleanBoundaryTraversalReport2, BooleanBoundaryTraversalStatus,
};
pub use bulge::BulgeVertex2;
pub use classify::{Classification, LineSide, UncertaintyReason};
pub use contour::{Contour2, ContourPointLocation, FillRule};
pub use curve_string::{CurveString2, CurveStringIntersection};
pub use error::{CurveError, CurveResult};
pub use events::{
    ContourIntersection, ContourIntersectionSet, ContourOperand, ContourOverlapIntersection,
    ContourPointIntersection, ContourUncertainIntersection,
};
pub use facts::{
    Bezier2Facts, BezierDegree, CircularArc2Facts, CurveStringFacts, LineSeg2Facts, Point2Facts,
    RationalQuadraticBezier2Facts, RegionFacts, Segment2Facts, SegmentKind, SegmentKindCounts,
};
pub use finite_projection::{
    FinitePolyline2, FiniteProjectionCertificate, FiniteProjectionOptions, FiniteRegionProfile2,
    FiniteRegionProjection2, FiniteRegionProjectionCertificate, finite_polyline_vertex_centroid,
    finite_ring_signed_area,
};
pub use fragment::{ContourFragment, ContourFragmentSet};
pub use intersect::{
    ArcArcIntersection, ArcArcIntersectionPoint, CircleCircleRelation, IntersectionKind,
    LineArcIntersection, LineArcIntersectionPoint, LineArcOrder, LineCircleRelation,
    LineLineIntersection, ParamRange, SegmentIntersection,
};
pub use offset::OffsetCap;
pub use point::Point2;
pub use policy::{CurvePolicy, NumericMode, Tolerance};
pub use prepared::{
    PreparedCircularArc2, PreparedContourView2, PreparedCurveStringView2, PreparedLineSeg2,
    PreparedRegionView2, PreparedSegment2,
};
pub use rational_bezier::{RationalQuadraticBezier2, RationalQuadraticConicKind};
pub use reconstruct::{
    FiniteContourImport2, FiniteCurveStringImport2, FiniteImportCertificate,
    PolylineReconstructionOptions,
};
pub use region::{Region2, RegionContourProfile, RegionPointLocation, RegionView2};
pub use region_events::{
    RegionContourIntersection, RegionContourKey, RegionContourRole, RegionIntersectionSet,
    RegionSide,
};
pub use region_fragments::{RegionContourFragments, RegionFragmentSet};
pub use region_nesting::{
    BoundaryContourNestingAuditReport2, BoundaryContourNestingReport2, BoundaryContourNestingStatus,
};
pub use segment::{CircularArc2, LineSeg2, Segment2};
pub use split::{ContourSplitMap, ContourSplitMarkers, SegmentSplitMarker, SegmentSplitPoint};
pub use transform::Similarity2;
#[cfg(feature = "triangulation")]
pub use triangulation::{FiniteTriangle2, triangulate_finite_rings};

pub use hyperreal::Rational;
pub use hyperreal::{Real, RealSign, SymbolicDependencyMask, ZeroKnowledge as ZeroStatus};

#[cfg(feature = "predicates")]
pub use hyperlimit::PredicatePolicy;

#[cfg(test)]
mod tests {
    use super::*;

    fn s(value: i32) -> Real {
        value.into()
    }

    fn p(x: i32, y: i32) -> Point2 {
        Point2::new(s(x), s(y))
    }

    fn topology_policy() -> CurvePolicy {
        CurvePolicy::certified()
    }

    #[test]
    fn line_segment_rejects_zero_length() {
        let err = LineSeg2::try_new(p(1, 2), p(1, 2))
            .expect_err("zero-length segment should be rejected");
        assert_eq!(err, CurveError::ZeroLengthLine);
    }

    #[test]
    fn line_segment_interpolates_midpoint() {
        let line = LineSeg2::try_new(p(0, 0), p(2, 4)).unwrap();
        let midpoint = line.point_at(Real::try_from(0.5_f64).unwrap());
        assert_eq!(midpoint, p(1, 2));
    }

    #[test]
    fn bulge_zero_constructs_line_segment() {
        let segment = Segment2::from_bulge(p(0, 0), p(2, 0), s(0)).unwrap();
        assert!(matches!(segment, Segment2::Line(_)));
    }

    #[test]
    fn positive_semicircle_bulge_constructs_arc() {
        let segment = Segment2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
        let Segment2::Arc(arc) = segment else {
            panic!("semicircle bulge should construct an arc");
        };

        assert_eq!(arc.center(), &p(1, 0));
        assert_eq!(arc.radius_squared(), s(1));
        assert!(!arc.is_clockwise());
        assert_eq!(arc.bulge(), Some(&s(1)));
    }

    #[test]
    fn negative_semicircle_bulge_constructs_clockwise_arc() {
        let segment = Segment2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap();
        let Segment2::Arc(arc) = segment else {
            panic!("semicircle bulge should construct an arc");
        };

        assert_eq!(arc.center(), &p(1, 0));
        assert!(arc.is_clockwise());
        assert_eq!(arc.bulge(), Some(&s(-1)));
    }

    #[test]
    fn bulge_vertex_builds_segment_to_next_vertex() {
        let a = BulgeVertex2::new(p(0, 0), s(1));
        let b = BulgeVertex2::new(p(2, 0), s(0));
        let segment = a.segment_to(&b).unwrap();
        assert!(matches!(segment, Segment2::Arc(_)));
    }

    #[test]
    fn curve_string_rejects_empty_segment_list() {
        let err = CurveString2::try_new(Vec::new())
            .expect_err("empty checked curve string should be rejected");
        assert_eq!(err, CurveError::EmptyCurveString);
    }

    #[test]
    fn curve_string_rejects_disconnected_segments() {
        let first = Segment2::Line(LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap());
        let second = Segment2::Line(LineSeg2::try_new(p(2, 0), p(3, 0)).unwrap());
        let err = CurveString2::try_new(vec![first, second])
            .expect_err("disconnected segments should be rejected");
        assert_eq!(err, CurveError::DisconnectedCurveString);
    }

    #[test]
    fn curve_string_builds_from_bulge_vertices() {
        let vertices = [
            BulgeVertex2::new(p(0, 0), s(0)),
            BulgeVertex2::new(p(1, 0), s(1)),
            BulgeVertex2::new(p(3, 0), s(0)),
        ];
        let curve = CurveString2::from_bulge_vertices(&vertices).unwrap();

        assert_eq!(curve.len(), 2);
        assert_eq!(curve.start(), Some(&p(0, 0)));
        assert_eq!(curve.end(), Some(&p(3, 0)));
        assert!(matches!(curve.segments()[0], Segment2::Line(_)));
        assert!(matches!(curve.segments()[1], Segment2::Arc(_)));
    }

    #[test]
    fn contour_signed_area_accumulates_line_segments_exactly() {
        let contour = Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(p(0, 0), s(0)),
            BulgeVertex2::new(p(2, 0), s(0)),
            BulgeVertex2::new(p(2, 3), s(0)),
            BulgeVertex2::new(p(0, 3), s(0)),
        ])
        .unwrap();

        assert_eq!(contour.signed_area().unwrap(), Some(Real::from(6_i8)));
    }

    #[test]
    fn contour_signed_area_accumulates_bulge_arc_segments() {
        let contour = Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(p(1, 0), s(1)),
            BulgeVertex2::new(p(-1, 0), s(0)),
        ])
        .unwrap();

        assert_eq!(
            contour.signed_area().unwrap(),
            Some((Real::pi() / Real::from(2_i8)).unwrap())
        );
    }

    #[test]
    fn region_filled_area_uses_material_minus_hole_roles() {
        let material = Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(p(0, 0), s(0)),
            BulgeVertex2::new(p(4, 0), s(0)),
            BulgeVertex2::new(p(4, 4), s(0)),
            BulgeVertex2::new(p(0, 4), s(0)),
        ])
        .unwrap();
        let clockwise_hole = Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(p(1, 1), s(0)),
            BulgeVertex2::new(p(1, 3), s(0)),
            BulgeVertex2::new(p(3, 3), s(0)),
            BulgeVertex2::new(p(3, 1), s(0)),
        ])
        .unwrap();
        let region = Region2::new(vec![material], vec![clockwise_hole]);

        assert_eq!(
            region.filled_area(&topology_policy()).unwrap(),
            Classification::Decided(Some(Real::from(12_i8)))
        );
    }

    #[test]
    fn curve_string_rejects_too_few_bulge_vertices() {
        let vertices = [BulgeVertex2::new(p(0, 0), s(0))];
        let err = CurveString2::from_bulge_vertices(&vertices)
            .expect_err("open curve string needs at least two vertices");
        assert_eq!(err, CurveError::InsufficientVertices);
    }

    #[test]
    fn curve_string_intersections_collect_line_line_event() {
        let a = CurveString2::try_new(vec![Segment2::Line(
            LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap(),
        )])
        .unwrap();
        let b = CurveString2::try_new(vec![Segment2::Line(
            LineSeg2::try_new(p(1, -1), p(1, 1)).unwrap(),
        )])
        .unwrap();

        let intersections = a.intersect_curve_string(&b, &topology_policy()).unwrap();
        assert_eq!(intersections.len(), 1);
        assert_eq!(intersections[0].a_segment_index, 0);
        assert_eq!(intersections[0].b_segment_index, 0);

        let SegmentIntersection::LineLine(LineLineIntersection::Point { point, kind, .. }) =
            &intersections[0].relation
        else {
            panic!("expected line-line curve-string event");
        };
        assert_eq!(point, &p(1, 0));
        assert_eq!(*kind, IntersectionKind::Crossing);
    }

    #[test]
    fn curve_string_intersections_collect_line_arc_event() {
        let line_curve = CurveString2::try_new(vec![Segment2::Line(
            LineSeg2::try_new(p(1, -2), p(1, 2)).unwrap(),
        )])
        .unwrap();
        let arc_curve = CurveString2::try_new(vec![Segment2::Arc(
            CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
        )])
        .unwrap();

        let intersections = line_curve
            .intersect_curve_string(&arc_curve, &topology_policy())
            .unwrap();
        assert_eq!(intersections.len(), 1);

        let SegmentIntersection::LineArc {
            order,
            result: LineArcIntersection::Point(hit),
        } = &intersections[0].relation
        else {
            panic!("expected line-arc curve-string event");
        };
        assert_eq!(*order, LineArcOrder::LineThenArc);
        assert_eq!(hit.point, p(1, -1));
    }

    #[test]
    fn curve_string_intersections_drop_empty_segment_pairs() {
        let a = CurveString2::try_new(vec![Segment2::Line(
            LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap(),
        )])
        .unwrap();
        let b = CurveString2::try_new(vec![Segment2::Line(
            LineSeg2::try_new(p(0, 1), p(1, 1)).unwrap(),
        )])
        .unwrap();

        let intersections = a.intersect_curve_string(&b, &topology_policy()).unwrap();
        assert!(intersections.is_empty());
    }

    #[test]
    fn line_side_classifies_left_right_and_on() {
        let line = LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap();
        assert_eq!(
            line.classify_point(&p(1, 1), &topology_policy()),
            Classification::Decided(LineSide::Left)
        );
        assert_eq!(
            line.classify_point(&p(1, -1), &topology_policy()),
            Classification::Decided(LineSide::Right)
        );
        assert_eq!(
            line.classify_point(&p(1, 0), &topology_policy()),
            Classification::Decided(LineSide::On)
        );
    }

    #[test]
    fn line_line_intersection_crosses_at_point() {
        let a = LineSeg2::try_new(p(0, 0), p(2, 2)).unwrap();
        let b = LineSeg2::try_new(p(0, 2), p(2, 0)).unwrap();
        let intersection = a.intersect_line(&b, &topology_policy()).unwrap();

        let LineLineIntersection::Point {
            point,
            a_param,
            b_param,
            kind,
        } = intersection
        else {
            panic!("expected one point intersection");
        };

        let half = Real::try_from(0.5_f64).unwrap();
        assert_eq!(point, p(1, 1));
        assert_eq!(a_param, half);
        assert_eq!(b_param, Real::try_from(0.5_f64).unwrap());
        assert_eq!(kind, IntersectionKind::Crossing);
    }

    #[test]
    fn line_line_intersection_detects_endpoint_touch() {
        let a = LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap();
        let b = LineSeg2::try_new(p(1, 0), p(1, 1)).unwrap();
        let intersection = a.intersect_line(&b, &topology_policy()).unwrap();

        let LineLineIntersection::Point { point, kind, .. } = intersection else {
            panic!("expected endpoint point intersection");
        };

        assert_eq!(point, p(1, 0));
        assert_eq!(kind, IntersectionKind::Endpoint);
    }

    #[test]
    fn line_line_intersection_detects_collinear_overlap() {
        let a = LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap();
        let b = LineSeg2::try_new(p(2, 0), p(6, 0)).unwrap();
        let intersection = a.intersect_line(&b, &topology_policy()).unwrap();

        let LineLineIntersection::Overlap {
            segment,
            a_range,
            b_range,
        } = intersection
        else {
            panic!("expected overlap");
        };

        assert_eq!(segment.start(), &p(2, 0));
        assert_eq!(segment.end(), &p(4, 0));
        assert_eq!(a_range.start(), &Real::try_from(0.5_f64).unwrap());
        assert_eq!(a_range.end(), &s(1));
        assert_eq!(b_range.start(), &s(0));
        assert_eq!(b_range.end(), &Real::try_from(0.5_f64).unwrap());
    }

    #[test]
    fn line_line_intersection_detects_parallel_disjoint() {
        let a = LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap();
        let b = LineSeg2::try_new(p(0, 1), p(1, 1)).unwrap();
        assert_eq!(
            a.intersect_line(&b, &topology_policy()).unwrap(),
            LineLineIntersection::None
        );
    }

    #[test]
    fn arc_sweep_classifies_positive_bulge_semicircle() {
        let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
        assert_eq!(
            arc.contains_sweep_point(&p(1, -1), &topology_policy()),
            Classification::Decided(true)
        );
        assert_eq!(
            arc.contains_sweep_point(&p(1, 1), &topology_policy()),
            Classification::Decided(false)
        );
        assert_eq!(
            arc.contains_sweep_point(&p(0, 0), &topology_policy()),
            Classification::Decided(true)
        );
    }

    #[test]
    fn arc_sweep_classifies_negative_bulge_semicircle() {
        let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap();
        assert_eq!(
            arc.contains_sweep_point(&p(1, 1), &topology_policy()),
            Classification::Decided(true)
        );
        assert_eq!(
            arc.contains_sweep_point(&p(1, -1), &topology_policy()),
            Classification::Decided(false)
        );
        assert_eq!(
            arc.contains_sweep_point(&p(2, 0), &topology_policy()),
            Classification::Decided(true)
        );
    }

    #[test]
    fn line_arc_intersection_keeps_only_points_inside_sweep() {
        let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
        let line = LineSeg2::try_new(p(1, -2), p(1, 2)).unwrap();
        let intersection = line.intersect_arc(&arc, &topology_policy()).unwrap();

        let LineArcIntersection::Point(hit) = intersection else {
            panic!("expected one line-arc hit");
        };

        assert_eq!(hit.point, p(1, -1));
        assert_eq!(hit.line_param, Real::try_from(0.25_f64).unwrap());
        assert_eq!(hit.kind, IntersectionKind::Crossing);
    }

    #[test]
    fn line_arc_intersection_keeps_clockwise_sweep() {
        let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap();
        let line = LineSeg2::try_new(p(1, -2), p(1, 2)).unwrap();
        let intersection = line.intersect_arc(&arc, &topology_policy()).unwrap();

        let LineArcIntersection::Point(hit) = intersection else {
            panic!("expected one line-arc hit");
        };

        assert_eq!(hit.point, p(1, 1));
        assert_eq!(hit.line_param, Real::try_from(0.75_f64).unwrap());
        assert_eq!(hit.kind, IntersectionKind::Crossing);
    }

    #[test]
    fn line_arc_intersection_detects_tangent() {
        let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
        let line = LineSeg2::try_new(p(0, -1), p(2, -1)).unwrap();
        let intersection = line.intersect_arc(&arc, &topology_policy()).unwrap();

        let LineArcIntersection::Point(hit) = intersection else {
            panic!("expected tangent hit");
        };

        assert_eq!(hit.point, p(1, -1));
        assert_eq!(hit.kind, IntersectionKind::Tangent);
    }

    #[test]
    fn line_arc_intersection_rejects_circle_hit_outside_sweep() {
        let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
        let line = LineSeg2::try_new(p(0, 1), p(2, 1)).unwrap();
        assert_eq!(
            line.intersect_arc(&arc, &topology_policy()).unwrap(),
            LineArcIntersection::None
        );
    }

    #[test]
    fn line_arc_intersection_detects_two_endpoint_hits() {
        let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
        let line = LineSeg2::try_new(p(-1, 0), p(3, 0)).unwrap();
        let intersection = line.intersect_arc(&arc, &topology_policy()).unwrap();

        let LineArcIntersection::TwoPoints { first, second } = intersection else {
            panic!("expected two endpoint hits");
        };

        assert_eq!(first.point, p(0, 0));
        assert_eq!(first.kind, IntersectionKind::Endpoint);
        assert_eq!(second.point, p(2, 0));
        assert_eq!(second.kind, IntersectionKind::Endpoint);
    }

    #[test]
    fn arc_arc_intersection_detects_one_filtered_crossing() {
        let a = CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap();
        let b = CircularArc2::try_from_center(p(3, 0), p(13, 0), p(8, 0), true).unwrap();

        let intersection = a.intersect_arc(&b, &topology_policy()).unwrap();
        let ArcArcIntersection::Point(hit) = intersection else {
            panic!("expected one filtered arc-arc hit");
        };

        assert_eq!(hit.point, p(4, 3));
        assert_eq!(hit.kind, IntersectionKind::Crossing);
    }

    #[test]
    fn arc_arc_intersection_detects_tangent() {
        let a = CircularArc2::try_from_center(p(0, -5), p(0, 5), p(0, 0), false).unwrap();
        let b = CircularArc2::try_from_center(p(10, 5), p(10, -5), p(10, 0), false).unwrap();

        let intersection = a.intersect_arc(&b, &topology_policy()).unwrap();
        let ArcArcIntersection::Point(hit) = intersection else {
            panic!("expected tangent arc-arc hit");
        };

        assert_eq!(hit.point, p(5, 0));
        assert_eq!(hit.kind, IntersectionKind::Tangent);
    }

    #[test]
    fn arc_arc_intersection_detects_two_endpoint_hits() {
        let a = CircularArc2::try_from_center(p(4, 3), p(4, -3), p(0, 0), true).unwrap();
        let b = CircularArc2::try_from_center(p(4, -3), p(4, 3), p(8, 0), true).unwrap();

        let intersection = a.intersect_arc(&b, &topology_policy()).unwrap();
        let ArcArcIntersection::TwoPoints { first, second } = intersection else {
            panic!("expected two endpoint arc-arc hits");
        };

        assert_eq!(first.point, p(4, 3));
        assert_eq!(first.kind, IntersectionKind::Endpoint);
        assert_eq!(second.point, p(4, -3));
        assert_eq!(second.kind, IntersectionKind::Endpoint);
    }

    #[test]
    fn arc_arc_intersection_detects_disjoint_circles() {
        let a = CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap();
        let b = CircularArc2::try_from_center(p(17, 0), p(7, 0), p(12, 0), false).unwrap();

        assert_eq!(
            a.intersect_arc(&b, &topology_policy()).unwrap(),
            ArcArcIntersection::None
        );
    }

    #[test]
    fn arc_arc_intersection_reports_same_circle_overlap() {
        let a = CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap();
        let b = CircularArc2::try_from_center(p(0, 5), p(0, -5), p(0, 0), false).unwrap();

        let intersection = a.intersect_arc(&b, &topology_policy()).unwrap();
        let ArcArcIntersection::Overlap {
            segment,
            a_range,
            b_range,
        } = intersection
        else {
            panic!("expected same-circle arc overlap");
        };

        assert_eq!(segment.start(), &p(0, 5));
        assert_eq!(segment.end(), &p(-5, 0));
        assert_eq!(a_range.start(), &Real::try_from(0.5_f64).unwrap());
        assert_eq!(a_range.end(), &s(1));
        assert_eq!(b_range.start(), &s(0));
        assert_eq!(b_range.end(), &Real::try_from(0.5_f64).unwrap());
    }

    #[test]
    fn arc_arc_intersection_reports_reversed_same_circle_overlap() {
        let a = CircularArc2::try_from_center(p(0, 0), p(2, 0), p(1, 0), false).unwrap();
        let b = CircularArc2::try_from_center(p(2, 0), p(0, 0), p(1, 0), true).unwrap();

        let intersection = a.intersect_arc(&b, &topology_policy()).unwrap();
        let ArcArcIntersection::Overlap {
            segment,
            a_range,
            b_range,
        } = intersection
        else {
            panic!("expected reversed same-circle arc overlap");
        };

        assert_eq!(segment.start(), &p(0, 0));
        assert_eq!(segment.end(), &p(2, 0));
        assert_eq!(a_range.start(), &s(0));
        assert_eq!(a_range.end(), &s(1));
        assert_eq!(b_range.start(), &s(1));
        assert_eq!(b_range.end(), &s(0));
    }

    #[test]
    fn arc_arc_intersection_reports_same_circle_endpoint_only_pair() {
        let a = CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap();
        let b = CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), true).unwrap();

        let intersection = a.intersect_arc(&b, &topology_policy()).unwrap();
        let ArcArcIntersection::TwoPoints { first, second } = intersection else {
            panic!("expected same-circle endpoint-only pair");
        };

        assert_eq!(first.point, p(5, 0));
        assert_eq!(first.kind, IntersectionKind::Endpoint);
        assert_eq!(second.point, p(-5, 0));
        assert_eq!(second.kind, IntersectionKind::Endpoint);
    }

    #[test]
    fn segment_intersection_dispatches_line_line() {
        let a = Segment2::Line(LineSeg2::try_new(p(0, 0), p(2, 2)).unwrap());
        let b = Segment2::Line(LineSeg2::try_new(p(0, 2), p(2, 0)).unwrap());
        let intersection = a.intersect_segment(&b, &topology_policy()).unwrap();

        let SegmentIntersection::LineLine(LineLineIntersection::Point { point, kind, .. }) =
            intersection
        else {
            panic!("expected dispatched line-line point");
        };

        assert_eq!(point, p(1, 1));
        assert_eq!(kind, IntersectionKind::Crossing);
    }

    #[test]
    fn segment_intersection_dispatches_line_arc_with_order() {
        let line = Segment2::Line(LineSeg2::try_new(p(1, -2), p(1, 2)).unwrap());
        let arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap());
        let intersection = line.intersect_segment(&arc, &topology_policy()).unwrap();

        let SegmentIntersection::LineArc {
            order,
            result: LineArcIntersection::Point(hit),
        } = intersection
        else {
            panic!("expected dispatched line-arc point");
        };

        assert_eq!(order, LineArcOrder::LineThenArc);
        assert_eq!(hit.point, p(1, -1));
    }

    #[test]
    fn segment_intersection_dispatches_arc_line_with_order() {
        let arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap());
        let line = Segment2::Line(LineSeg2::try_new(p(1, -2), p(1, 2)).unwrap());
        let intersection = arc.intersect_segment(&line, &topology_policy()).unwrap();

        let SegmentIntersection::LineArc {
            order,
            result: LineArcIntersection::Point(hit),
        } = intersection
        else {
            panic!("expected dispatched arc-line point");
        };

        assert_eq!(order, LineArcOrder::ArcThenLine);
        assert_eq!(hit.point, p(1, -1));
    }

    #[test]
    fn segment_intersection_dispatches_arc_arc() {
        let a = Segment2::Arc(
            CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap(),
        );
        let b =
            Segment2::Arc(CircularArc2::try_from_center(p(3, 0), p(13, 0), p(8, 0), true).unwrap());
        let intersection = a.intersect_segment(&b, &topology_policy()).unwrap();

        let SegmentIntersection::ArcArc(ArcArcIntersection::Point(hit)) = intersection else {
            panic!("expected dispatched arc-arc point");
        };

        assert_eq!(hit.point, p(4, 3));
        assert_eq!(hit.kind, IntersectionKind::Crossing);
    }
}
