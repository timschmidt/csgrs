#![allow(
    clippy::large_enum_variant,
    clippy::result_large_err,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

//! Curve primitives built on the hyper geometry stack.
//!
//! This crate starts with a line/circular-arc core using [`hyperreal::Real`]
//! coordinates and the `hyperlimit` predicate policy model. The core topology
//! follows the robust-computation principle of deciding predicates before
//! branching.

mod arc_bezier;
mod bbox;
mod bezier;
mod bezier_algebraic_image;
mod bezier_arrangement;
mod bezier_fit;
mod bezier_flatten;
mod bezier_metric;
mod bezier_moment;
mod bezier_offset;
mod bezier_parameter;
mod bezier_region;
mod bezier_retained_measure;
mod bezier_retained_overlap;
mod bezier_split;
mod bezier_split_endpoint;
mod bezier_tangent_order;
mod bezier_topology;
mod boolean_op;
mod bspline;
mod bulge;
mod classify;
mod contour;
mod contour_regularize;
mod curve;
mod curve_intersection;
mod curve_path_intersection;
mod curve_region_boolean;
mod curve_region_trim;
mod curve_string;
mod error;
mod events;
mod facts;
mod finite_projection;
mod fragment;
#[cfg(feature = "hershey")]
pub mod hershey;
#[cfg(feature = "hershey")]
mod hershey_data;
mod intersect;
mod nurbs;
mod nurbs_interpolation;
mod offset;
mod point;
mod policy;
mod polynomial_spline;
mod prepared;
mod rational_bezier;
mod rational_bezier_general;
mod reconstruct;
mod region;
mod region_events;
mod region_nesting;
mod retained_status;
mod segment;
mod self_intersect;
mod spline_periodic;
mod split;
mod straight_skeleton;
#[cfg(feature = "svg")]
mod svg;
mod transform;
mod translation_obstacle;
#[cfg(feature = "triangulation")]
mod triangulation;

pub use arc_bezier::{CircularArcBezierDecomposition2, CircularArcBezierSpan2};
pub use bbox::Aabb2;
pub use bezier::{BezierEndpoint, CubicBezier2, EndpointTangent2, QuadraticBezier2};
pub use bezier_algebraic_image::{
    BezierAlgebraicCoordinateImage, BezierAlgebraicImageStatus, BezierAlgebraicPointImage2,
    BezierAlgebraicRationalCoordinateImage, BezierAlgebraicTangentImage2,
    RationalBezierAlgebraicPointImage2, RationalBezierAlgebraicTangentImage2,
};
pub use bezier_arrangement::{
    BezierArrangementChain2, BezierArrangementFragment2, BezierArrangementGraph2,
    BezierArrangementTraversal2,
};
pub use bezier_fit::{
    BezierFitBoundKind, BezierFitCertificate, BezierFitErrorMetric, BezierLineFitRelation,
    BezierLineImageFitRelation, BezierPointFitRelation, BezierPointImageFitRelation,
    CertifiedBezierLineFit2, CertifiedBezierLineImage2, CertifiedBezierLineImageOffset2,
    CertifiedBezierLineOffset2, CertifiedBezierPointFit2, CertifiedBezierPointImage2,
};
pub use bezier_flatten::{
    BezierFlatteningCertificate, BezierFlatteningOptions, CertifiedBezierPolyline2,
    CertifiedCurvePolyline2,
};
pub use bezier_metric::{BezierArcLengthParameterRegion2, BezierLengthBounds2};
pub use bezier_moment::{BezierAreaMomentPrefixSums2, BezierAreaMoments2, BezierAreaPrefixSums2};
#[cfg(feature = "predicates")]
pub use bezier_offset::BezierAlgebraicChordPairPoint2;
pub use bezier_offset::{
    BezierAlgebraicChord2, BezierAlgebraicCuspSemicircleFragment2, BezierOffsetCandidate2,
    BezierOffsetPreflight2, BezierOffsetRisk, BezierParallel2, BezierParallelApproximationCurve2,
    BezierParallelIncidence2, BezierParallelIntersectionCandidates2,
    BezierParallelIntersectionContact2, BezierParallelIntersectionParameterComponent2,
    BezierParallelIntersectionSet2, BezierParallelPairIntersectionCandidates2,
    BezierParallelPairIntersectionContact2, BezierParallelPairIntersectionParameterComponent2,
    BezierParallelPairIntersectionSet2, BezierParallelSingularityAnalysis2, BezierParallelSource2,
    BezierParallelVerificationOptions, Blend2dCubicQuadraticReduction2,
    Blend2dQuadraticOffsetCandidate2, CertifiedBezierParallelApproximation2,
    CertifiedBezierParallelPath2, CertifiedBezierParallelSpan2, CertifiedCurvePathParallel2,
    CertifiedPythagoreanHodographOffset2, LevienCubicOffsetCandidate2,
};
#[cfg(feature = "predicates")]
pub use bezier_offset::{
    BezierAlgebraicChordParallelPoint2, BezierAlgebraicCuspChordDerivedPoint2,
    BezierAlgebraicCuspChordPoint2,
};
pub use bezier_parameter::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierParameterRange2, BezierRootIsolationResult2,
    BezierRootIsolationTrace2,
};
pub use bezier_region::{
    BezierBoundaryLoop2, BezierRegion2, CurveRegion2, CurveRegionArrangement2,
    CurveRegionBoundaryContourBuildResult2, CurveRegionBoundaryLoop2,
    CurveRegionCertifiedParallelLoopEvidence2, CurveRegionCertifiedParallelOffsetEvidence2,
    CurveRegionCertifiedParallelOffsetResult2, CurveRegionCertifiedSegmentationEvidence2,
    CurveRegionCertifiedSegmentationResult2, CurveRegionFragmentSource2,
    CurveRegionLineRoleEvidence2, CurveRegionLoopRole, CurveRegionNativeContourView2,
    CurveRegionNestingRoleEvidence2, CurveRegionProfile2, CurveRegionSegmentationLoopEvidence2,
    CurveRegionSegmentedOffsetEvidence2, CurveRegionSegmentedOffsetResult2,
    CurveRegionSignedAreaRoleEvidence2,
};
pub use bezier_retained_measure::{
    BezierRetainedCurveEnvelope2, BezierRetainedEndpointEnvelope2, BezierRetainedEnvelopeSourceKind,
};
pub use bezier_retained_overlap::{
    BezierRetainedLineOverlapSplit2, BezierRetainedLinearOverlapSplit2,
    BezierRetainedLinearOverlapSplitGraph2, BezierRetainedLinearOverlapTraversal2,
    BezierRetainedOverlap2, BezierRetainedOverlapEvidence2, BezierRetainedOverlapExtent2,
    BezierRetainedOverlapOrientation2, BezierRetainedOverlapRefinedFragment2,
    BezierRetainedOverlapRelation2, BezierRetainedOverlapTraversal2,
    BezierRetainedRationalOverlapSplit2, BezierRetainedRationalOverlapSplitGraph2,
    BezierRetainedRationalOverlapTraversal2, BezierRetainedResolvedLinearOverlap2,
    BezierRetainedResolvedRationalOverlap2,
};
pub use bezier_split::{
    BezierParallelFragment2, BezierSplitFragment2, BezierSplitMaterialization2, BezierSubcurve2,
    CurveRegionParameter2, CurveRegionParameterRange2,
};
pub use bezier_split_endpoint::{
    BezierAlgebraicEndpointImage2, BezierEndpointPointImage2, BezierEndpointTangentImage2,
};
pub use bezier_tangent_order::{
    BezierAlgebraicSameTangentOrderEvidence, BezierAlgebraicSameTangentOrderStatus,
    BezierAlgebraicScalarSignEvidence, BezierAlgebraicTangentOrderEvidence,
    BezierAlgebraicTangentOrderStatus, BezierAlgebraicTangentVector2,
    BezierAlgebraicTangentVectorEvidence, BezierAlgebraicTangentVectorStatus,
    BezierTangentTurnOrdering2, compare_algebraic_same_tangent_second_order,
    compare_algebraic_same_tangent_third_order, compare_algebraic_tangent_turn_from_base,
};
pub use bezier_topology::{
    Axis2, BezierCurveIntersectionPoint, BezierCurveIntersectionRegion, BezierCurveRelation,
    BezierCuspClassification, BezierGraphContact, BezierInflectionClassification,
    BezierLineContact, BezierLineContactKind, BezierLineContactRelation,
    BezierLineCrossingDirection, BezierLineRelation, BezierMonotoneGraphContactOrder,
    BezierMonotoneGraphOrder, BezierMonotoneSpan,
};
pub use boolean_op::BooleanOp;
pub use bspline::{
    PolynomialBSplineBezierExtraction2, PolynomialBSplineCurve2, RationalBSplineBezierExtraction2,
    RationalBSplineCurve2, RationalBSplineNativeTopologyEvidence2, RationalBezierSpan2,
    RationalBezierSpanTopologyEvidence2, RationalBezierSpanTopologyPath2,
    RationalQuadraticBSplineBezierExtraction2, RationalQuadraticBSplineCurve2,
    RetainedBSplineSpanFactEvidence2, RetainedBSplineSpanFacts2, RetainedSpanAxisMonotonicity,
    RetainedSpanWeightDomainEvidence2,
};
pub use bulge::BulgeVertex2;
pub use classify::{Classification, LineSide, UncertaintyReason};
pub use contour::{Contour2, ContourPointLocation, FillRule};
pub use curve::{
    Curve2, CurveCornerMode2, CurveCornerNoSolution2, CurveCornerSolutions2, CurveDerivative2,
    CurveFamily2, CurveGeometry2, CurveParameterDomain2, CurveParameterSide2, CurvePath2,
    CurvePathView2, CurveSpanRange2, CurveView2, NativeBezierBoundaryLoop2, NativeBezierFragment2,
};
pub use curve_intersection::{
    CurveIntersectionContact2, CurveIntersectionOverlap2, CurveIntersectionPairBlocker2,
    CurveIntersectionPairBlockerKind2, CurveIntersectionParameter2, CurveIntersectionResult2,
    CurveIntersectionTopology2,
};
pub use curve_path_intersection::{
    CurveBoundaryInteriorSide2, CurvePathBooleanFragment2, CurvePathBooleanFragmentAction2,
    CurvePathBooleanOperand2, CurvePathBooleanSelection2, CurvePathBooleanSelections2,
    CurvePathIntersectionBlocker2, CurvePathIntersectionContact2, CurvePathIntersectionOverlap2,
    CurvePathIntersectionResult2, CurvePathIntersectionTopology2, CurvePathOverlapAction2,
    CurvePathOverlapResolution2, CurvePathSplit2,
};
pub use curve_region_boolean::{
    CurveRegionBooleanResults2, CurveRegionCarrierRef2, CurveRegionIntersectionBlocker2,
    CurveRegionIntersectionContact2, CurveRegionIntersectionOverlap2,
    CurveRegionIntersectionResult2,
};
pub use curve_region_trim::{
    CurveRegionBoundaryContact2, CurveRegionBoundaryKind2, CurveRegionTrimFragment2,
};
pub use curve_string::{
    CurveString2, CurveStringEndpoint2, CurveStringIntersection, CurveStringLinkKind2,
    CurveStringTrimPoint2,
};
pub use error::{
    CurveError, CurveOperation2, CurveResult, ExactCurveBlocker, ExactCurveError, ExactCurveResult,
};
pub use events::{
    ContourIntersection, ContourIntersectionSet, ContourOperand, ContourOverlapIntersection,
    ContourPointIntersection, ContourUncertainIntersection,
};
pub use facts::{
    Bezier2Facts, BezierDegree, CircularArc2Facts, CurveStringFacts, LineSeg2Facts, Point2Facts,
    RationalQuadraticBezier2Facts, RegionFacts, Segment2Facts, SegmentKind, SegmentKindCounts,
};
pub use finite_projection::{
    FinitePolyline2, FiniteProjectionOptions, FiniteRegionProfile2, FiniteRegionProjection2,
    finite_polyline_vertex_centroid, finite_ring_signed_area, try_finite_polyline_vertex_centroid,
    try_finite_ring_signed_area,
};
pub use fragment::{ContourFragment, ContourFragmentSet};
pub use intersect::{
    ArcArcIntersection, ArcArcIntersectionPoint, CircleCircleRelation, IntersectionKind,
    LineArcIntersection, LineArcIntersectionPoint, LineArcOrder, LineCircleRelation,
    LineLineIntersection, ParamRange, SegmentIntersection,
};
pub use nurbs::{
    NurbsBezierDecomposition2, NurbsBezierSpanView2, NurbsCurve2, NurbsDegreeElevation2,
    NurbsElevatedBezierSpan2, NurbsNativeSpanView2,
};
pub use offset::{OffsetCap, OffsetCornerStyle2};
pub use point::Point2;
pub use policy::{CurveCertainty, CurveContext, CurveOutcome, CurvePreviewOptions};
pub use polynomial_spline::{
    PolynomialSplineBezierDecomposition2, PolynomialSplineBezierSpanView2, PolynomialSplineCurve2,
};
pub use rational_bezier::{RationalQuadraticBezier2, RationalQuadraticConicKind};
pub use rational_bezier_general::{
    RationalBezier2, RationalBezierIntersectionCandidates2, RationalBezierIntersectionContact2,
    RationalBezierIntersectionContacts2, RationalBezierIntersectionOverlap2,
    RationalBezierIntersectionPointEvidence2, RationalBezierIntersectionTopology2,
    RationalBezierOverlapOrientation2, RationalBezierPointIncidence2,
};
pub use reconstruct::PolylineReconstructionOptions;
#[doc(hidden)]
pub use region::LineArcRegion2;
pub use region::{RegionContourProfile, RegionPointLocation, RegionView2};
pub use region_events::{
    RegionContourIntersection, RegionContourKey, RegionContourRole, RegionIntersectionSet,
    RegionSide,
};
pub use region_nesting::{
    RegionArrangement2, RegionArrangementSummary2, RegionBoundaryContourBuildEvidence2,
    RegionBoundaryContourBuildPredicatePath2, RegionBoundaryContourBuildResult2,
    RegionBoundaryContourBuildStage2, RegionBoundaryContourRole2,
    RegionBoundaryContourRoleEvidence2, RegionLineSegmentArrangedEndpoint2,
    RegionLineSegmentArrangedSourceEvidence2, RegionLineSegmentEndpointGraphPredicatePath2,
    RegionLineSegmentRegionBuildStage2, RegionLineSegmentRingAssemblyPredicatePath2,
    RegionLineSegmentRingSourceEvidence2, RegionLineSegmentSplitIntersectionEvidence2,
    RegionLineSegmentSplitPredicatePath2,
};
pub use retained_status::RetainedTopologyStatus;
pub use segment::{CircularArc2, LineSeg2, Segment2};
pub use spline_periodic::SplinePeriodicity2;
pub use split::{ContourSplitMap, ContourSplitMarkers, SegmentSplitMarker, SegmentSplitPoint};
pub use straight_skeleton::{
    StraightSkeleton2, StraightSkeletonAffineTime2, StraightSkeletonArc2,
    StraightSkeletonArcGeometry2, StraightSkeletonArcKind2, StraightSkeletonBlocker2,
    StraightSkeletonConicBranch2, StraightSkeletonCurveFamilySupport2,
    StraightSkeletonGeneratedBisector2, StraightSkeletonGlobalContactEvent2,
    StraightSkeletonGlobalContactKind2, StraightSkeletonImplicitConic2,
    StraightSkeletonLocalArcEvent2, StraightSkeletonLocalArcEventKind2, StraightSkeletonNode2,
    StraightSkeletonNodeKind2, StraightSkeletonResult2, StraightSkeletonSpliceEvent2,
    StraightSkeletonStage2, StraightSkeletonSupportProvenance2,
    StraightSkeletonTrajectoryGeometry2, StraightSkeletonTrajectoryKind2,
    StraightSkeletonVertexTrajectory2,
};
#[cfg(feature = "svg")]
pub use svg::{
    SvgError, SvgGeometry2, SvgOptions, SvgResult, SvgSubpath2, export_svg_document,
    export_svg_document_with_options, import_svg_document, import_svg_document_with_options,
    parse_svg_path_data,
};
pub use transform::Similarity2;
pub use translation_obstacle::{
    TranslationObstacle2, TranslationObstacleBlocker2, TranslationObstacleEvidence2,
    TranslationObstacleOperand2, translation_obstacle_convex,
};
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

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn p(x: i32, y: i32) -> Point2 {
        Point2::new(s(x), s(y))
    }

    fn topology_policy() -> CurveContext {
        CurveContext::STRICT
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
    fn angular_arc_splits_retain_root_lineage_across_nested_edits() {
        let arc = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap();
        let Classification::Decided((first, second)) = arc
            .split_at_sweep_fraction(&q(1, 3), &topology_policy())
            .unwrap()
        else {
            panic!("exact rational sweep fraction must decide");
        };
        let sqrt_three_over_two = (s(3).sqrt().unwrap() / s(2)).unwrap();
        let expected_first_cut = Point2::new(sqrt_three_over_two.clone(), q(1, 2));
        assert_eq!(
            crate::classify::is_zero(
                &first.end().distance_squared(&expected_first_cut),
                &topology_policy(),
            ),
            Some(true)
        );
        assert_eq!(first.end(), second.start());

        let Classification::Decided((nested_first, nested_second)) = second
            .split_at_sweep_fraction(&q(1, 2), &topology_policy())
            .unwrap()
        else {
            panic!("nested exact rational sweep fraction must decide");
        };
        let expected_second_cut = Point2::new(q(1, 2), sqrt_three_over_two);
        assert_eq!(
            crate::classify::is_zero(
                &nested_first.end().distance_squared(&expected_second_cut),
                &topology_policy(),
            ),
            Some(true)
        );
        assert_eq!(nested_first.end(), nested_second.start());
    }

    #[test]
    fn angular_fraction_maps_directly_to_native_rational_arc_parameter() {
        let arc = CircularArc2::try_from_center(p(2, 0), p(0, 2), p(0, 0), false).unwrap();
        let fraction = q(1, 2);
        let Classification::Decided(parameter) = arc
            .parameter_at_sweep_fraction(&fraction, &topology_policy())
            .unwrap()
        else {
            panic!("an exact interior angular fraction must map exactly");
        };
        let Classification::Decided(angular_point) = arc
            .point_at_sweep_fraction(&fraction, &topology_policy())
            .unwrap()
        else {
            panic!("an exact interior angular fraction must evaluate exactly");
        };
        let expected_midpoint = Point2::new(s(2).sqrt().unwrap(), s(2).sqrt().unwrap());
        assert_eq!(
            crate::classify::is_zero(
                &angular_point.distance_squared(&expected_midpoint),
                &topology_policy(),
            ),
            Some(true)
        );
        let expected_parameter = s(2).sqrt().unwrap() - s(1);
        assert_eq!(
            crate::classify::compare_reals(&parameter, &expected_parameter, &topology_policy()),
            Some(std::cmp::Ordering::Equal)
        );
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
        let region = LineArcRegion2::new(vec![material], vec![clockwise_hole]);

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
    fn line_line_intersection_retains_symbolic_shared_endpoint_parameters() {
        let shared = Point2::new(Real::pi(), Real::one());
        let a = LineSeg2::try_new(p(0, 0), shared.clone()).unwrap();
        let b =
            LineSeg2::try_new(shared.clone(), Point2::new(Real::pi(), Real::from(2_i8))).unwrap();

        let LineLineIntersection::Point {
            point,
            a_param,
            b_param,
            kind,
        } = a.intersect_line(&b, &topology_policy()).unwrap()
        else {
            panic!("expected symbolic endpoint point intersection");
        };

        assert_eq!(point, shared);
        assert_eq!(a_param, Real::one());
        assert_eq!(b_param, Real::zero());
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
    fn independently_constructed_exact_offsets_retain_parallel_relation() {
        let angle = (Real::pi() / Real::from(20_u8)).unwrap();
        let source = LineSeg2::try_new(
            Point2::new(Real::from(15_u8), Real::zero()),
            Point2::new(
                Real::from(15_u8) * angle.clone().cos(),
                Real::from(15_u8) * angle.sin(),
            ),
        )
        .unwrap();
        let left = source.offset_left(Real::one()).unwrap();
        let right = source.offset_left(-Real::one()).unwrap();

        assert_eq!(
            left.intersect_line(&right, &topology_policy()).unwrap(),
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
        assert_eq!(hit.arc_param, q(1, 2));
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
        assert_eq!(hit.arc_param, q(1, 2));
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
        assert_eq!(hit.line_param, q(1, 2));
        assert_eq!(hit.arc_param, q(1, 2));
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
        assert_eq!(first.arc_param, s(0));
        assert_eq!(first.kind, IntersectionKind::Endpoint);
        assert_eq!(second.point, p(2, 0));
        assert_eq!(second.arc_param, s(1));
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
        let hit_fraction = (q(3, 4).atan().unwrap() / Real::pi()).unwrap();
        assert_eq!(hit.a_param, hit_fraction);
        assert_eq!(hit.b_param, hit.a_param);
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
        assert_eq!(hit.a_param, q(1, 2));
        assert_eq!(hit.b_param, q(1, 2));
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
        assert_eq!(first.a_param, s(0));
        assert_eq!(first.b_param, s(1));
        assert_eq!(first.kind, IntersectionKind::Endpoint);
        assert_eq!(second.point, p(4, -3));
        assert_eq!(second.a_param, s(1));
        assert_eq!(second.b_param, s(0));
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
    fn arc_arc_intersection_evidence_same_circle_overlap() {
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
    fn arc_arc_intersection_evidence_reversed_same_circle_overlap() {
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
    fn arc_arc_intersection_evidence_same_circle_endpoint_only_pair() {
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
