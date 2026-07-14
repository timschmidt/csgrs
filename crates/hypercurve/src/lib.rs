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
//! follows the robust-computation principle of deciding predicates before branching; see
//! Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast Robust
//! Geometric Predicates" (*Discrete & Computational Geometry* 18(3), 305-363,
//! 1997).

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
mod boolean;
mod boolean_boundary;
mod bspline;
mod bulge;
mod classify;
mod contour;
mod curve;
mod curve_intersection;
mod curve_path_intersection;
mod curve_region_boolean;
mod curve_string;
mod error;
mod events;
mod facts;
mod finite_projection;
mod fragment;
mod intersect;
mod nurbs;
mod nurbs_interpolation;
mod offset;
mod point;
mod policy;
mod polynomial_spline;
mod prepared;
mod prepared_boolean;
mod rational_bezier;
mod rational_bezier_general;
mod reconstruct;
mod region;
mod region_boolean;
mod region_events;
mod region_fragments;
mod region_nesting;
mod retained_curve;
mod retained_import;
mod retained_status;
mod segment;
mod self_intersect;
mod spline_periodic;
mod split;
#[cfg(feature = "svg")]
mod svg_io;
mod transform;
#[cfg(feature = "triangulation")]
mod triangulation;

pub use arc_bezier::{CircularArcBezierDecomposition2, CircularArcBezierSpan2};
pub use bbox::Aabb2;
pub use bezier::{
    BezierEndpoint, BezierInterpolationReplayPath2, BezierInterpolationSolvePath2, CubicBezier2,
    CubicBezierHermiteInterpolationReport2, CubicBezierHermiteInterpolationResult2,
    CubicBezierHermiteInterpolationStage2, EndpointTangent2, QuadraticBezier2,
    QuadraticBezierMidpointInterpolationReport2, QuadraticBezierMidpointInterpolationResult2,
    QuadraticBezierMidpointInterpolationStage2, QuadraticBezierPointInterpolationReport2,
    QuadraticBezierPointInterpolationResult2, QuadraticBezierPointInterpolationStage2,
};
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
};
pub use bezier_metric::{BezierArcLengthParameterRegion2, BezierLengthBounds2};
pub use bezier_moment::{BezierAreaMomentPrefixSums2, BezierAreaMoments2, BezierAreaPrefixSums2};
pub use bezier_offset::{BezierOffsetCandidate2, BezierOffsetPreflight2, BezierOffsetRisk};
pub use bezier_parameter::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierParameterRange2,
};
pub use bezier_region::{
    BezierBoundaryLoop2, BezierRegion2, CurveRegion2, CurveRegionBoundaryLoop2,
    CurveRegionFragmentProvenance2, CurveRegionFragmentSource2, CurveRegionLineRoleReport2,
    CurveRegionLoopRole, CurveRegionNestingRoleReport2, CurveRegionSignedAreaRoleReport2,
};
pub use bezier_retained_measure::{
    BezierRetainedCurveEnvelope2, BezierRetainedEndpointEnvelope2, BezierRetainedEnvelopeSourceKind,
};
pub use bezier_retained_overlap::{
    BezierRetainedLineOverlapSplit2, BezierRetainedLinearOverlapSplit2,
    BezierRetainedLinearOverlapSplitGraph2, BezierRetainedLinearOverlapTraversal2,
    BezierRetainedOverlap2, BezierRetainedOverlapExtent2, BezierRetainedOverlapOrientation2,
    BezierRetainedOverlapRefinedFragment2, BezierRetainedOverlapRelation2,
    BezierRetainedOverlapReport2, BezierRetainedOverlapTraversal2,
    BezierRetainedRationalOverlapSplit2, BezierRetainedRationalOverlapSplitGraph2,
    BezierRetainedRationalOverlapTraversal2, BezierRetainedResolvedLinearOverlap2,
    BezierRetainedResolvedRationalOverlap2,
};
pub use bezier_split::{BezierSplitFragment2, BezierSplitMaterialization2, BezierSubcurve2};
pub use bezier_split_endpoint::{
    BezierAlgebraicEndpointImage2, BezierEndpointPointImage2, BezierEndpointTangentImage2,
};
pub use bezier_tangent_order::{
    BezierAlgebraicSameTangentOrderReport, BezierAlgebraicSameTangentOrderStatus,
    BezierAlgebraicScalarSignReport, BezierAlgebraicTangentOrderReport,
    BezierAlgebraicTangentOrderStatus, BezierAlgebraicTangentVector2,
    BezierAlgebraicTangentVectorReport, BezierAlgebraicTangentVectorStatus,
    BezierTangentTurnOrdering2, compare_algebraic_same_tangent_second_order,
    compare_algebraic_same_tangent_third_order, compare_algebraic_tangent_turn_from_base,
};
pub use bezier_topology::{
    Axis2, BezierCurveIntersectionPoint, BezierCurveIntersectionRegion, BezierCurveRelation,
    BezierCuspClassification, BezierGraphContact, BezierInflectionClassification,
    BezierLineContact, BezierLineContactKind, BezierLineContactRelation, BezierLineRelation,
    BezierMonotoneGraphContactOrder, BezierMonotoneGraphOrder, BezierMonotoneSpan,
};
pub use boolean::{
    BooleanBoundaryFragmentEmissionReport2, BooleanBoundaryFragmentEmissionResult2,
    BooleanBoundaryFragmentEmissionStage2, BooleanDirectedFragmentReport2, BooleanFragmentAction,
    BooleanFragmentClassification, BooleanFragmentSelection, BooleanFragmentSelectionReport2,
    BooleanFragmentSelectionResult2, BooleanFragmentSelectionStage2, BooleanOp,
};
pub use boolean_boundary::{
    BooleanBoundaryChain, BooleanBoundaryChainAssemblyReport2, BooleanBoundaryChainAssemblyResult2,
    BooleanBoundaryChainAssemblyStage2, BooleanBoundaryChainSet,
    BooleanBoundaryContourTransferReport2, BooleanBoundaryContourTransferResult2,
    BooleanBoundaryContourTransferStage2, BooleanBoundaryFragmentSet, BooleanBoundaryLoop,
    BooleanBoundaryLoopConstructionReport2, BooleanBoundaryLoopConstructionResult2,
    BooleanBoundaryLoopConstructionStage2, BooleanBoundaryLoopExtractionReport2,
    BooleanBoundaryLoopExtractionResult2, BooleanBoundaryLoopExtractionStage2,
    BooleanBoundaryLoopSet, BooleanBoundaryOutputFragmentReport2, DirectedBooleanFragment,
};
pub use bspline::{
    PolynomialBSplineBezierExtraction2, PolynomialBSplineCurve2, RationalBSplineBezierExtraction2,
    RationalBSplineCurve2, RationalBSplineNativeTopologyReport2, RationalBezierSpan2,
    RationalBezierSpanTopologyPath2, RationalBezierSpanTopologyReport2,
    RationalQuadraticBSplineBezierExtraction2, RationalQuadraticBSplineCurve2,
    RetainedBSplineSpanFactReport2, RetainedBSplineSpanFacts2, RetainedSpanAxisMonotonicity,
    RetainedSpanWeightDomainReport2,
};
pub use bulge::BulgeVertex2;
pub use classify::{Classification, LineSide, UncertaintyReason};
pub use contour::{
    Contour2, ContourChamferReport2, ContourChamferResult2, ContourChamferStage2,
    ContourClosurePredicatePath2, ContourClosureReport2, ContourClosureResult2,
    ContourClosureStage2, ContourFilletReport2, ContourFilletResult2, ContourFilletStage2,
    ContourLineMergePredicatePath2, ContourLineMergeReport2, ContourLineMergeResult2,
    ContourLineMergeSpanReport2, ContourLineMergeStage2, ContourPointLocation, FillRule,
};
pub use curve::{
    Curve2, CurveDerivative2, CurveFamily2, CurveGeometry2, CurveParameterDomain2,
    CurveParameterSide2, CurvePath2, CurvePathView2, CurveSource2, CurveSpanProvenance2,
    CurveView2, NativeBezierBoundaryLoop2, NativeBezierFragment2,
};
pub use curve_intersection::{
    CurveIntersectionContact2, CurveIntersectionOverlap2, CurveIntersectionPairBlocker2,
    CurveIntersectionPairBlockerKind2, CurveIntersectionParameter2, CurveIntersectionReport2,
    CurveIntersectionTopology2, PreparedCurveIntersection2,
};
pub use curve_path_intersection::{
    CurveBoundaryInteriorSide2, CurvePathBooleanFragment2, CurvePathBooleanFragmentAction2,
    CurvePathBooleanOperand2, CurvePathBooleanSelection2, CurvePathIntersectionBlocker2,
    CurvePathIntersectionContact2, CurvePathIntersectionOverlap2, CurvePathIntersectionReport2,
    CurvePathIntersectionTopology2, CurvePathOverlapAction2, CurvePathOverlapResolution2,
    CurvePathSplit2, PreparedCurvePathIntersection2,
};
pub use curve_region_boolean::PreparedCurveRegionBoolean2;
pub use curve_string::{
    ConnectedCurveString2, CurveString2, CurveStringChamferInputPath2,
    CurveStringChamferPredicatePath2, CurveStringChamferReport2, CurveStringChamferResult2,
    CurveStringChamferStage2, CurveStringConnectOutputSegmentReport2,
    CurveStringConnectPredicatePath2, CurveStringConnectReport2, CurveStringConnectSource2,
    CurveStringConnectStage2, CurveStringCurveTrimHit2, CurveStringCurveTrimQueryPath2,
    CurveStringCurveTrimReport2, CurveStringCurveTrimResult2, CurveStringCurveTrimStage2,
    CurveStringDeduplicatePredicatePath2, CurveStringDeduplicateReport2,
    CurveStringDeduplicateResult2, CurveStringDeduplicateRetainedSegmentReport2,
    CurveStringDeduplicateStage2, CurveStringEndpoint2,
    CurveStringEndpointConnectionPredicatePath2, CurveStringEndpointConnectionReport2,
    CurveStringEndpointConnectionStatus2, CurveStringExtendPredicatePath2,
    CurveStringExtendReport2, CurveStringExtendResult2, CurveStringExtendStage2,
    CurveStringFilletInputPath2, CurveStringFilletPredicatePath2, CurveStringFilletReport2,
    CurveStringFilletResult2, CurveStringFilletStage2, CurveStringIntersection,
    CurveStringIntersectionPredicatePath2, CurveStringIntersectionPreparedCacheReport2,
    CurveStringIntersectionQueryPath2, CurveStringIntersectionReport2,
    CurveStringIntersectionResult2, CurveStringLineMergePredicatePath2,
    CurveStringLineMergeReport2, CurveStringLineMergeResult2, CurveStringLineMergeSpanReport2,
    CurveStringLineMergeStage2, CurveStringLinkAttemptReport2, CurveStringLinkAttemptResult2,
    CurveStringLinkKind2, CurveStringLinkOutputSegmentReport2, CurveStringLinkPredicatePath2,
    CurveStringLinkReport2, CurveStringLinkSourceInput2, CurveStringLinkStage2,
    CurveStringOrderedLinkPredicatePath2, CurveStringOrderedLinkReport2,
    CurveStringOrderedLinkStage2, CurveStringOrderedLinkStepReport2,
    CurveStringPreparedCacheAudit2, CurveStringPreparedCacheFreshness2,
    CurveStringRegionTrimBoundaryPredicatePath2, CurveStringRegionTrimHit2,
    CurveStringRegionTrimIntervalReport2, CurveStringRegionTrimPreparedCacheReport2,
    CurveStringRegionTrimQueryPath2, CurveStringRegionTrimReport2, CurveStringRegionTrimResult2,
    CurveStringRegionTrimStage2, CurveStringReversedDuplicatePairReport2,
    CurveStringTrimInputPath2, CurveStringTrimPoint2, CurveStringTrimPredicatePath2,
    CurveStringTrimReport2, CurveStringTrimResult2, CurveStringTrimSegmentReport2,
    LinkedCurveString2, OrderedLinkedCurveString2, RegionTrimPreparedCacheAudit2,
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
pub use nurbs_interpolation::{
    NurbsInterpolation2, NurbsInterpolationParameterization2, NurbsInterpolationReport2,
    NurbsInterpolationSolvePath2,
};
pub use offset::{
    ContourOffsetReport2, ContourOffsetResult2, ContourOffsetStage2, CurveStringOffsetReport2,
    CurveStringOffsetResult2, CurveStringOffsetStage2, CurveStringOutlineOffsetReport2,
    CurveStringOutlineOffsetResult2, CurveStringOutlineOffsetStage2, OffsetCap,
    OffsetConstructionPath2, OutlineCapConstructionPath2,
};
pub use point::Point2;
pub use policy::{CurvePolicy, NumericMode, Tolerance};
pub use polynomial_spline::{
    PolynomialSplineBezierDecomposition2, PolynomialSplineBezierSpanView2, PolynomialSplineCurve2,
};
pub use prepared::{
    PreparedCircularArc2, PreparedContourView2, PreparedCurveStringView2, PreparedLineSeg2,
    PreparedRegionView2, PreparedSegment2,
};
pub use rational_bezier::{RationalQuadraticBezier2, RationalQuadraticConicKind};
pub use rational_bezier_general::{
    PreparedRationalBezierIntersection2, RationalBezier2, RationalBezierIntersectionCandidates2,
    RationalBezierIntersectionContact2, RationalBezierIntersectionContacts2,
    RationalBezierIntersectionOverlap2, RationalBezierIntersectionPointEvidence2,
    RationalBezierIntersectionTopology2, RationalBezierOverlapOrientation2,
    RationalBezierPointIncidence2,
};
pub use reconstruct::{
    ContourPolylineReconstructionResult2, CurveStringPolylineReconstructionResult2,
    FiniteContourImport2, FiniteCurveStringImport2, PolylineReconstructionOptions,
    PolylineReconstructionReport2, PolylineReconstructionSegmentReport2,
};
pub use region::{Region2, RegionContourProfile, RegionPointLocation, RegionView2};
pub use region_boolean::{
    RegionBooleanBoundaryContourSourcePath2, RegionBooleanBoundaryPredicatePath2,
    RegionBooleanPipelineReport2, RegionBooleanPreparedCacheReport2, RegionBooleanQueryPath2,
    RegionBooleanReport2, RegionBooleanResult2, RegionBooleanSharedBoundaryResolution2,
    RegionBooleanStage2, RegionPreparedCacheAudit2, RegionPreparedCacheFreshness2,
};
pub use region_events::{
    RegionContourIntersection, RegionContourKey, RegionContourRole, RegionIntersectionSet,
    RegionSide,
};
pub use region_fragments::{
    RegionContourFragmentReport2, RegionContourFragments, RegionContourOutputFragmentReport2,
    RegionFragmentBuildPredicatePath2, RegionFragmentBuildReport2, RegionFragmentBuildResult2,
    RegionFragmentBuildStage2, RegionFragmentSet,
};
pub use region_nesting::{
    ExactCurveArrangementArrangedEndpointDegree2, ExactCurveArrangementArrangedFragment2,
    ExactCurveArrangementOutputRoleAssignment2, ExactCurveArrangementSourceAabbStatus2,
    ExactCurveArrangementSourceEndpoint2, ExactCurveArrangementSourceSegmentFact2,
    ExactCurveArrangementSplitCandidateAabbStatus2, ExactCurveArrangementSplitCandidatePair2,
    ExactCurveArrangementSplitRelationClass2, ExactCurveArrangementSummary2, RegionArrangement2,
    RegionArrangementReport2, RegionBoundaryContourBuildPredicatePath2,
    RegionBoundaryContourBuildReport2, RegionBoundaryContourBuildResult2,
    RegionBoundaryContourBuildStage2, RegionBoundaryContourRole2, RegionBoundaryContourRoleReport2,
    RegionLineSegmentArrangedEndpoint2, RegionLineSegmentArrangedSourceReport2,
    RegionLineSegmentEndpointGraphPredicatePath2, RegionLineSegmentRegionBuildStage2,
    RegionLineSegmentRingAssemblyPredicatePath2, RegionLineSegmentRingSourceReport2,
    RegionLineSegmentSplitIntersectionReport2, RegionLineSegmentSplitPredicatePath2,
};
pub use retained_curve::{
    RetainedCurveCacheSummary2, RetainedCurveFamily2, RetainedCurveIdentity2,
    RetainedCurveProfile2, RetainedEndpointEvidence2, RetainedParameterDomain1,
    RetainedTrimDirection, RetainedTrimInterval1,
};
pub use retained_import::{
    RetainedImportFormat2, RetainedImportRecord2, RetainedImportTopology2, RetainedSourceTolerance2,
};
pub use retained_status::RetainedTopologyStatus;
pub use segment::{CircularArc2, LineSeg2, Segment2};
pub use self_intersect::{
    SelfContactPredicatePath2, SelfContactPreparedCacheFreshness2, SelfContactPreparedCacheReport2,
    SelfContactReport2, SelfContactResult2,
};
pub use spline_periodic::SplinePeriodicity2;
pub use split::{ContourSplitMap, ContourSplitMarkers, SegmentSplitMarker, SegmentSplitPoint};
#[cfg(feature = "svg")]
pub use svg_io::{
    SvgContourImportReport2, SvgContourImportResult2, SvgPathExportCurveReport2,
    SvgPathExportReport2, SvgPathExportResult2, SvgPathExportSegmentReport2, SvgPathExportTarget2,
    SvgPathImportReport2, SvgPathImportResult2, SvgRegionImportReport2, SvgRegionImportResult2,
    import_svg_contour_path_data_with_report, import_svg_path_data_with_report,
    import_svg_region_path_data_with_report, retained_svg_import_record,
};
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

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
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
