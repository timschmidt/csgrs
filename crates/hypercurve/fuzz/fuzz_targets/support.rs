//! Shared native hypercurve fuzz helpers.
//!
//! The fuzz targets intentionally generate small but relationship-rich inputs, then assert the
//! same invariants as the integration tests: valid signed shape bins, fresh spatial indexes, and
//! sampled set-membership semantics. The byte reader returns deterministic defaults when input is
//! exhausted so minimized cases remain short and replayable.

#![allow(dead_code)]

use hypercurve::{
    Aabb2 as HAabb2, ArcArcIntersection as HArcArcIntersection,
    ArcArcIntersectionPoint as HArcArcIntersectionPoint, Axis2 as HAxis2,
    BezierBooleanAlgebraicParameterCarrierReport2 as HBezierBooleanAlgebraicParameterCarrierReport2,
    BezierBooleanAlgebraicParameterCarrierStatus as HBezierBooleanAlgebraicParameterCarrierStatus,
    BezierBooleanAlgebraicParameterHandoffReport2 as HBezierBooleanAlgebraicParameterHandoffReport2,
    BezierBooleanAlgebraicParameterOrderingReport2 as HBezierBooleanAlgebraicParameterOrderingReport2,
    BezierBooleanAlgebraicParameterOrderingStatus as HBezierBooleanAlgebraicParameterOrderingStatus,
    BezierBooleanAlgebraicParameterReadinessReport2 as HBezierBooleanAlgebraicParameterReadinessReport2,
    BezierBooleanAlgebraicParameterRole as HBezierBooleanAlgebraicParameterRole,
    BezierBooleanAlgebraicSplitBridgeReport2 as HBezierBooleanAlgebraicSplitBridgeReport2,
    BezierBooleanAssemblyReadinessStatus as HBezierBooleanAssemblyReadinessStatus,
    BezierBooleanBatchHandoffReport2 as HBezierBooleanBatchHandoffReport2,
    BezierBooleanConstructionReadinessStatus as HBezierBooleanConstructionReadinessStatus,
    BezierBooleanCubicFragmentReport2 as HBezierBooleanCubicFragmentReport2,
    BezierBooleanFragmentConstructionStatus as HBezierBooleanFragmentConstructionStatus,
    BezierBooleanFragmentEndpointTangents2 as HBezierBooleanFragmentEndpointTangents2,
    BezierBooleanFragmentLocatorInputReport2 as HBezierBooleanFragmentLocatorInputReport2,
    BezierBooleanFragmentLocatorInputStatus as HBezierBooleanFragmentLocatorInputStatus,
    BezierBooleanFragmentOwnershipLocation as HBezierBooleanFragmentOwnershipLocation,
    BezierBooleanLoopAssemblyPlanReport2 as HBezierBooleanLoopAssemblyPlanReport2,
    BezierBooleanLoopAssemblyPlanStatus as HBezierBooleanLoopAssemblyPlanStatus,
    BezierBooleanLoopClosureReport2 as HBezierBooleanLoopClosureReport2,
    BezierBooleanLoopContainmentCertificationReport2 as HBezierBooleanLoopContainmentCertificationReport2,
    BezierBooleanLoopContainmentCertificationStatus as HBezierBooleanLoopContainmentCertificationStatus,
    BezierBooleanLoopContainmentQueryResultReport2 as HBezierBooleanLoopContainmentQueryResultReport2,
    BezierBooleanLoopContainmentQueryResultStatus as HBezierBooleanLoopContainmentQueryResultStatus,
    BezierBooleanLoopGraphMultiCycleWalkReport2 as HBezierBooleanLoopGraphMultiCycleWalkReport2,
    BezierBooleanLoopGraphMultiCycleWalkStatus as HBezierBooleanLoopGraphMultiCycleWalkStatus,
    BezierBooleanLoopGraphSuccessorFact2 as HBezierBooleanLoopGraphSuccessorFact2,
    BezierBooleanLoopGraphTraversalReport2 as HBezierBooleanLoopGraphTraversalReport2,
    BezierBooleanLoopNestingDepthFact2 as HBezierBooleanLoopNestingDepthFact2,
    BezierBooleanOutputLoopReport2 as HBezierBooleanOutputLoopReport2,
    BezierBooleanOverlapBridgeFact2 as HBezierBooleanOverlapBridgeFact2,
    BezierBooleanOverlapEvent2 as HBezierBooleanOverlapEvent2,
    BezierBooleanOverlapResolutionReport2 as HBezierBooleanOverlapResolutionReport2,
    BezierBooleanOwnedTraversalStep2 as HBezierBooleanOwnedTraversalStep2,
    BezierBooleanOwnershipFact2 as HBezierBooleanOwnershipFact2,
    BezierBooleanPathSchedulerReport2 as HBezierBooleanPathSchedulerReport2,
    BezierBooleanQuadraticFragmentReport2 as HBezierBooleanQuadraticFragmentReport2,
    BezierBooleanRationalQuadraticFragmentReport2 as HBezierBooleanRationalQuadraticFragmentReport2,
    BezierBooleanResultReport2 as HBezierBooleanResultReport2,
    BezierBooleanResultStatus as HBezierBooleanResultStatus,
    BezierBooleanTangentTurnPolicy as HBezierBooleanTangentTurnPolicy,
    BezierBooleanTraversalOperand as HBezierBooleanTraversalOperand,
    BezierBooleanTraversalPreconditionStatus as HBezierBooleanTraversalPreconditionStatus,
    BezierBooleanTraversalScheduleReport2 as HBezierBooleanTraversalScheduleReport2,
    BezierBooleanTraversalScheduleStatus as HBezierBooleanTraversalScheduleStatus,
    BezierBooleanTraversalStep2 as HBezierBooleanTraversalStep2,
    BezierCurveRelation as HBezierCurveRelation, BezierFitBoundKind as HBezierFitBoundKind,
    BezierFitErrorMetric as HBezierFitErrorMetric, BezierLineContactKind as HBezierLineContactKind,
    BezierLineContactRelation as HBezierLineContactRelation,
    BezierLineFitRelation as HBezierLineFitRelation,
    BezierLineImageFitRelation as HBezierLineImageFitRelation,
    BezierMonotoneGraphOrder as HBezierMonotoneGraphOrder,
    BezierMonotoneSpan as HBezierMonotoneSpan, BezierOffsetCandidate2 as HBezierOffsetCandidate2,
    BezierOffsetRisk as HBezierOffsetRisk,
    BezierPathRangeBatchReport2 as HBezierPathRangeBatchReport2,
    BezierPathRangeOrderReport2 as HBezierPathRangeOrderReport2,
    BezierPointFitRelation as HBezierPointFitRelation,
    BezierPointImageFitRelation as HBezierPointImageFitRelation,
    BezierSimplificationBoundKind as HBezierSimplificationBoundKind,
    BezierSimplificationErrorMetric as HBezierSimplificationErrorMetric,
    BooleanBoundaryAuditStatus as HBooleanBoundaryAuditStatus,
    BooleanBoundaryTraversalReport2 as HBooleanBoundaryTraversalReport2,
    BooleanBoundaryTraversalStatus as HBooleanBoundaryTraversalStatus,
    BooleanFragmentAction as HBooleanFragmentAction, BooleanOp as HBooleanOp,
    BooleanRegionAuditStatus as HBooleanRegionAuditStatus,
    BoundaryContourNestingStatus as HBoundaryContourNestingStatus, BulgeVertex2 as HBulgeVertex2,
    CircularArc2 as HCircularArc2, Classification as HClassification, Contour2 as HContour2,
    ContourOperand as HContourOperand, ContourPointLocation as HContourPointLocation,
    ContourSplitMap as HContourSplitMap, CubicBezier2 as HCubicBezier2,
    CurvePolicy as HCurvePolicy, CurveString2 as HCurveString2,
    CurveStringIntersection as HCurveStringIntersection, FillRule as HFillRule,
    LineArcIntersection as HLineArcIntersection,
    LineArcIntersectionPoint as HLineArcIntersectionPoint,
    LineLineIntersection as HLineLineIntersection, LineSeg2 as HLineSeg2, OffsetCap as HOffsetCap,
    ParamRange as HParamRange, Point2 as HPoint2,
    PolylineReconstructionOptions as HPolylineReconstructionOptions,
    QuadraticBezier2 as HQuadraticBezier2, RationalQuadraticBezier2 as HRationalQuadraticBezier2,
    Real as HReal, Region2 as HRegion2, RegionPointLocation as HRegionPointLocation,
    Segment2 as HSegment2, SegmentIntersection as HSegmentIntersection, Tolerance as HTolerance,
    UncertaintyReason as HUncertaintyReason,
};
use hypersolve::{
    AlgebraicRootKind as HAlgebraicRootKind,
    AlgebraicRootRefinementComparisonConfig as HAlgebraicRootRefinementComparisonConfig,
    AlgebraicRootRepresentation as HAlgebraicRootRepresentation,
    AlgebraicRootRepresentationReport as HAlgebraicRootRepresentationReport,
    AlgebraicRootRepresentationStatus as HAlgebraicRootRepresentationStatus,
    AlgebraicRootValidationReport as HAlgebraicRootValidationReport,
    AlgebraicRootValidationStatus as HAlgebraicRootValidationStatus,
    IsolatedRootInterval as HIsolatedRootInterval, SymbolId as HSymbolId,
};
use std::f64::consts::PI;

const H_GEOMETRY_EQ_EPS: f64 = 1e-6;

/// Deterministic byte reader for lightweight, shrinkable fuzz input decoding.
pub struct ByteReader<'a> {
    data: &'a [u8],
    index: usize,
}

impl<'a> ByteReader<'a> {
    /// Create a reader over the current fuzz input.
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, index: 0 }
    }

    /// Read one byte, returning zero after the input is exhausted.
    pub fn byte(&mut self) -> u8 {
        let byte = self.data.get(self.index).copied().unwrap_or(0);
        self.index = self.index.saturating_add(1);
        byte
    }

    /// Decode a boolean from the next byte.
    pub fn bool(&mut self) -> bool {
        self.byte() & 1 == 1
    }

    /// Decode an integer in an inclusive range.
    pub fn usize_range(&mut self, min: usize, max: usize) -> usize {
        min + usize::from(self.byte()) % (max - min + 1)
    }

    /// Decode an integer in an inclusive range.
    pub fn i32_range(&mut self, min: i32, max: i32) -> i32 {
        min + i32::from(self.byte()) % (max - min + 1)
    }

    /// Decode a bounded finite float from four bytes.
    pub fn f64_range(&mut self, min: f64, max: f64) -> f64 {
        let raw = u32::from_le_bytes([self.byte(), self.byte(), self.byte(), self.byte()]);
        let t = f64::from(raw) / f64::from(u32::MAX);
        min + (max - min) * t
    }
}

type HPoint = HPoint2;
type HRealValue = HReal;
type HContour = HContour2;
type HRegion = HRegion2;
type HSegment = HSegment2;

#[derive(Clone, Copy, Debug)]
struct HRect {
    xmin: f64,
    ymin: f64,
    xmax: f64,
    ymax: f64,
}

impl HRect {
    fn inset(self, x_margin: f64, y_margin: f64) -> Self {
        Self {
            xmin: self.xmin + x_margin,
            ymin: self.ymin + y_margin,
            xmax: self.xmax - x_margin,
            ymax: self.ymax - y_margin,
        }
    }

    fn width(self) -> f64 {
        self.xmax - self.xmin
    }

    fn height(self) -> f64 {
        self.ymax - self.ymin
    }
}

fn h_scalar(value: f64) -> HRealValue {
    HReal::try_from(value).unwrap()
}

fn h_scalar_i32(value: i32) -> HRealValue {
    value.into()
}

fn h_ratio(numerator: i32, denominator: i32) -> HRealValue {
    (HReal::from(numerator) / HReal::from(denominator)).unwrap()
}

fn h_point(x: f64, y: f64) -> HPoint {
    HPoint2::new(h_scalar(x), h_scalar(y))
}

fn h_point_i32(x: i32, y: i32) -> HPoint {
    HPoint2::new(h_scalar_i32(x), h_scalar_i32(y))
}

fn h_vertex(x: f64, y: f64, bulge: f64) -> HBulgeVertex2 {
    HBulgeVertex2::new(h_point(x, y), h_scalar(bulge))
}

fn h_vertex_i32(x: i32, y: i32, bulge: i32) -> HBulgeVertex2 {
    HBulgeVertex2::new(h_point_i32(x, y), h_scalar_i32(bulge))
}

fn h_policy() -> HCurvePolicy {
    HCurvePolicy::certified()
}

fn h_offset_policy() -> HCurvePolicy {
    HCurvePolicy::edge_preview(HTolerance::new(1e-8, 1e-8))
}

fn h_boolean_op(reader: &mut ByteReader<'_>) -> HBooleanOp {
    match reader.byte() & 3 {
        0 => HBooleanOp::Union,
        1 => HBooleanOp::Intersection,
        2 => HBooleanOp::Difference,
        _ => HBooleanOp::Xor,
    }
}

fn h_rect_from_bytes(reader: &mut ByteReader<'_>) -> HRect {
    let x = reader.f64_range(-64.0, 64.0);
    let y = reader.f64_range(-64.0, 64.0);
    let width = reader.f64_range(0.25, 40.0);
    let height = reader.f64_range(0.25, 40.0);
    HRect {
        xmin: x,
        ymin: y,
        xmax: x + width,
        ymax: y + height,
    }
}

fn h_rectangle_contour(rect: HRect) -> HContour {
    HContour2::from_bulge_vertices_with_fill_rule(
        &[
            h_vertex(rect.xmin, rect.ymin, 0.0),
            h_vertex(rect.xmax, rect.ymin, 0.0),
            h_vertex(rect.xmax, rect.ymax, 0.0),
            h_vertex(rect.xmin, rect.ymax, 0.0),
        ],
        HFillRule::NonZero,
    )
    .unwrap()
}

fn h_region_from_bytes(
    reader: &mut ByteReader<'_>,
    max_materials: usize,
    max_holes: usize,
) -> HRegion {
    let material_count = reader.usize_range(1, max_materials.max(1));
    let mut rects = Vec::with_capacity(material_count);
    let mut materials = Vec::with_capacity(material_count);
    for _ in 0..material_count {
        let rect = h_rect_from_bytes(reader);
        rects.push(rect);
        materials.push(h_rectangle_contour(rect));
    }

    let mut holes = Vec::new();
    if max_holes > 0 {
        let hole_count = reader.usize_range(0, max_holes);
        let outer = rects[0];
        for _ in 0..hole_count {
            let margin_x = reader.f64_range(outer.width() * 0.12, outer.width() * 0.42);
            let margin_y = reader.f64_range(outer.height() * 0.12, outer.height() * 0.42);
            let hole = outer.inset(margin_x, margin_y);
            if hole.width() > 1e-9 && hole.height() > 1e-9 {
                holes.push(h_rectangle_contour(hole));
            }
        }
    }

    HRegion2::new(materials, holes)
}

fn h_region_bounds(regions: &[&HRegion]) -> Option<(f64, f64, f64, f64)> {
    let policy = h_policy();
    regions
        .iter()
        .filter_map(|region| match HAabb2::from_region(region, &policy) {
            Ok(HClassification::Decided(bounds)) => Some((
                bounds.min_x().to_f64_approx()?,
                bounds.min_y().to_f64_approx()?,
                bounds.max_x().to_f64_approx()?,
                bounds.max_y().to_f64_approx()?,
            )),
            Ok(HClassification::Uncertain(_)) | Err(_) => None,
        })
        .fold(None, |acc, bounds| {
            Some(match acc {
                None => bounds,
                Some((min_x, min_y, max_x, max_y)) => (
                    min_x.min(bounds.0),
                    min_y.min(bounds.1),
                    max_x.max(bounds.2),
                    max_y.max(bounds.3),
                ),
            })
        })
}

fn h_region_membership(region: &HRegion, x: f64, y: f64) -> Option<bool> {
    match region.classify_point(&h_point(x, y), &h_policy()) {
        HClassification::Decided(HRegionPointLocation::Inside) => Some(true),
        HClassification::Decided(HRegionPointLocation::Outside) => Some(false),
        HClassification::Decided(HRegionPointLocation::Boundary)
        | HClassification::Uncertain(_) => None,
    }
}

fn h_expected_boolean(in_a: bool, in_b: bool, op: HBooleanOp) -> bool {
    match op {
        HBooleanOp::Union => in_a || in_b,
        HBooleanOp::Intersection => in_a && in_b,
        HBooleanOp::Difference => in_a && !in_b,
        HBooleanOp::Xor => in_a != in_b,
    }
}

fn h_assert_point_finite(point: &HPoint) {
    assert!(point.x().to_f64_approx().is_some_and(f64::is_finite));
    assert!(point.y().to_f64_approx().is_some_and(f64::is_finite));
}

fn h_assert_scalar_unit_interval(value: &HRealValue) {
    let value = value
        .to_f64_approx()
        .expect("fuzz scalar should be approximable");
    assert!(
        (-1e-8..=1.0 + 1e-8).contains(&value),
        "segment parameter out of range: {value}"
    );
}

fn h_scalar_f64(value: &HRealValue, context: &str) -> f64 {
    let value = value
        .to_f64_approx()
        .unwrap_or_else(|| panic!("{context}: scalar should be approximable"));
    assert!(value.is_finite(), "{context}: scalar should be finite");
    value
}

fn h_assert_scalar_approx_eq(left: &HRealValue, right: &HRealValue, context: &str) {
    let left = h_scalar_f64(left, context);
    let right = h_scalar_f64(right, context);
    let tolerance = H_GEOMETRY_EQ_EPS.max(H_GEOMETRY_EQ_EPS * left.abs().max(right.abs()));
    assert!(
        (left - right).abs() <= tolerance,
        "{context}: expected approximately equal scalars, left={left}, right={right}, tolerance={tolerance}"
    );
}

fn h_assert_point_approx_eq(left: &HPoint, right: &HPoint, context: &str) {
    h_assert_scalar_approx_eq(left.x(), right.x(), context);
    h_assert_scalar_approx_eq(left.y(), right.y(), context);
}

fn h_assert_param_range_approx_eq(left: &HParamRange, right: &HParamRange, context: &str) {
    h_assert_scalar_approx_eq(left.start(), right.start(), context);
    h_assert_scalar_approx_eq(left.end(), right.end(), context);
}

fn h_assert_line_segment_approx_eq(left: &HLineSeg2, right: &HLineSeg2, context: &str) {
    h_assert_point_approx_eq(left.start(), right.start(), context);
    h_assert_point_approx_eq(left.end(), right.end(), context);
}

fn h_assert_arc_segment_approx_eq(left: &HCircularArc2, right: &HCircularArc2, context: &str) {
    h_assert_point_approx_eq(left.start(), right.start(), context);
    h_assert_point_approx_eq(left.end(), right.end(), context);
    h_assert_point_approx_eq(left.center(), right.center(), context);
    h_assert_scalar_approx_eq(&left.radius_squared(), &right.radius_squared(), context);
    assert_eq!(left.is_clockwise(), right.is_clockwise(), "{context}");
}

fn h_assert_segment_approx_eq(left: &HSegment, right: &HSegment, context: &str) {
    match (left, right) {
        (HSegment2::Line(left), HSegment2::Line(right)) => {
            h_assert_line_segment_approx_eq(left, right, context);
        }
        (HSegment2::Arc(left), HSegment2::Arc(right)) => {
            h_assert_arc_segment_approx_eq(left, right, context);
        }
        _ => {
            panic!("{context}: expected matching segment variants, left={left:?}, right={right:?}")
        }
    }
}

fn h_assert_segment_finite(segment: &HSegment) {
    match segment {
        HSegment2::Line(line) => {
            h_assert_point_finite(line.start());
            h_assert_point_finite(line.end());
        }
        HSegment2::Arc(arc) => {
            h_assert_point_finite(arc.start());
            h_assert_point_finite(arc.end());
            h_assert_point_finite(arc.center());
            assert!(
                arc.radius_squared()
                    .to_f64_approx()
                    .is_some_and(f64::is_finite)
            );
        }
    }
}

fn h_assert_curve_string_finite(curve: &HCurveString2) {
    for segment in curve.segments() {
        h_assert_segment_finite(segment);
    }
}

fn h_assert_contour_finite(contour: &HContour) {
    h_assert_curve_string_finite(contour.curve_string());
}

fn h_assert_region_finite(region: &HRegion) {
    for contour in region
        .material_contours()
        .iter()
        .chain(region.hole_contours().iter())
    {
        h_assert_contour_finite(contour);
    }
}

fn h_assert_contour_boundary_sets_match(left: &[HContour], right: &[HContour]) {
    assert_eq!(
        left.len(),
        right.len(),
        "boundary contour sets should have equal cardinality"
    );
    let mut matched = vec![false; right.len()];
    for contour in left {
        let Some((index, _)) = right.iter().enumerate().find(|(index, candidate)| {
            !matched[*index] && contour.has_same_exact_boundary(candidate)
        }) else {
            panic!("boundary contour set is missing {contour:?}");
        };
        matched[index] = true;
    }
}

fn h_assert_region_semantics(a: &HRegion, b: &HRegion, result: &HRegion, op: HBooleanOp) {
    let Some((min_x, min_y, max_x, max_y)) = h_region_bounds(&[a, b, result]) else {
        return;
    };
    let width = (max_x - min_x).abs().max(1.0);
    let height = (max_y - min_y).abs().max(1.0);
    let fractions = [0.137, 0.311, 0.587, 0.829];

    for fx in fractions {
        for fy in fractions {
            let x = min_x + width * fx;
            let y = min_y + height * fy;
            let (Some(in_a), Some(in_b), Some(actual)) = (
                h_region_membership(a, x, y),
                h_region_membership(b, x, y),
                h_region_membership(result, x, y),
            ) else {
                continue;
            };
            assert_eq!(
                actual,
                h_expected_boolean(in_a, in_b, op),
                "hypercurve region boolean semantic mismatch: op={op:?}, point=({x}, {y})"
            );
        }
    }
}

/// Validates traversal accounting against explicit unresolved/shared/ready blockers.
///
/// This mirrors the invariants used by `BooleanBoundaryTraversalReport2` in
/// degenerate-contact-aware boolean dispatch (Yap, *Towards Exact Geometric
/// Computation*, 1997).
fn h_assert_traversal_report_invariants(
    report: &HBooleanBoundaryTraversalReport2,
    expected_status: HBooleanBoundaryTraversalStatus,
    expected_blocker: Option<HUncertaintyReason>,
) {
    assert_eq!(report.status, expected_status);
    assert_eq!(report.blocker_reason, expected_blocker);
    assert_eq!(
        report.is_ready(),
        matches!(
            expected_status,
            HBooleanBoundaryTraversalStatus::Empty | HBooleanBoundaryTraversalStatus::LoopsReady
        )
    );
    assert_eq!(
        report.classified_fragment_count,
        report.discarded_fragment_count
            + report.kept_source_direction_count
            + report.kept_reversed_count
            + report.unresolved_boundary_count
    );
    assert_eq!(
        report.directed_fragment_count,
        report.kept_source_direction_count + report.kept_reversed_count
    );
    if report.is_ready() {
        assert_eq!(report.open_chain_count, 0);
        assert_eq!(
            report.closed_chain_count + report.open_chain_count,
            report.assembled_chain_count
        );
    }
}

/// Asserts an audited boundary report status in the expected "usable decision" set.
fn h_assert_boundary_audit_is_valid(audit: &HBooleanBoundaryAuditStatus) {
    assert!(audit.is_valid());
    assert!(matches!(
        audit,
        HBooleanBoundaryAuditStatus::Empty
            | HBooleanBoundaryAuditStatus::Valid
            | HBooleanBoundaryAuditStatus::PointContact
    ));
}

/// Asserts an audited region report status in the expected "usable decision" set.
fn h_assert_region_audit_is_valid(audit: &HBooleanRegionAuditStatus) {
    assert!(matches!(
        audit,
        HBooleanRegionAuditStatus::Empty
            | HBooleanRegionAuditStatus::Valid
            | HBooleanRegionAuditStatus::PointContact
    ));
}

fn h_line_from_i32(start: (i32, i32), end: (i32, i32)) -> HLineSeg2 {
    HLineSeg2::try_new(h_point_i32(start.0, start.1), h_point_i32(end.0, end.1)).unwrap()
}

fn h_random_line(reader: &mut ByteReader<'_>) -> HSegment {
    let x = reader.i32_range(-64, 64);
    let y = reader.i32_range(-64, 64);
    let mut dx = reader.i32_range(-32, 32);
    let dy = reader.i32_range(-32, 32);
    if dx == 0 && dy == 0 {
        dx = 1;
    }
    HSegment2::Line(h_line_from_i32((x, y), (x + dx, y + dy)))
}

fn h_semicircle(reader: &mut ByteReader<'_>) -> HSegment {
    let cx = reader.i32_range(-32, 32);
    let cy = reader.i32_range(-32, 32);
    let radius = reader.i32_range(1, 24);
    let clockwise = reader.bool();
    HSegment2::from_bulge(
        h_point_i32(cx - radius, cy),
        h_point_i32(cx + radius, cy),
        h_scalar_i32(if clockwise { -1 } else { 1 }),
    )
    .unwrap()
}

fn h_segment_from_bytes(reader: &mut ByteReader<'_>) -> HSegment {
    if reader.bool() {
        h_random_line(reader)
    } else {
        h_semicircle(reader)
    }
}

fn h_validate_line_line(result: &HLineLineIntersection) {
    match result {
        HLineLineIntersection::None | HLineLineIntersection::Uncertain { .. } => {}
        HLineLineIntersection::Point {
            point,
            a_param,
            b_param,
            ..
        } => {
            h_assert_point_finite(point);
            h_assert_scalar_unit_interval(a_param);
            h_assert_scalar_unit_interval(b_param);
        }
        HLineLineIntersection::Overlap {
            segment,
            a_range,
            b_range,
        } => {
            h_assert_point_finite(segment.start());
            h_assert_point_finite(segment.end());
            h_assert_scalar_unit_interval(a_range.start());
            h_assert_scalar_unit_interval(a_range.end());
            h_assert_scalar_unit_interval(b_range.start());
            h_assert_scalar_unit_interval(b_range.end());
        }
    }
}

fn h_validate_line_arc(result: &HLineArcIntersection) {
    let validate_point = |point: &hypercurve::LineArcIntersectionPoint| {
        h_assert_point_finite(&point.point);
        h_assert_scalar_unit_interval(&point.line_param);
    };

    match result {
        HLineArcIntersection::None | HLineArcIntersection::Uncertain { .. } => {}
        HLineArcIntersection::Point(point) => validate_point(point),
        HLineArcIntersection::TwoPoints { first, second } => {
            validate_point(first);
            validate_point(second);
        }
    }
}

fn h_validate_arc_arc(result: &HArcArcIntersection) {
    let validate_point = |point: &hypercurve::ArcArcIntersectionPoint| {
        h_assert_point_finite(&point.point);
    };

    match result {
        HArcArcIntersection::None | HArcArcIntersection::Uncertain { .. } => {}
        HArcArcIntersection::Point(point) => validate_point(point),
        HArcArcIntersection::TwoPoints { first, second } => {
            validate_point(first);
            validate_point(second);
        }
        HArcArcIntersection::Overlap {
            segment,
            a_range,
            b_range,
        } => {
            h_assert_point_finite(segment.start());
            h_assert_point_finite(segment.end());
            h_assert_point_finite(segment.center());
            h_assert_scalar_unit_interval(a_range.start());
            h_assert_scalar_unit_interval(a_range.end());
            h_assert_scalar_unit_interval(b_range.start());
            h_assert_scalar_unit_interval(b_range.end());
        }
    }
}

fn h_validate_segment_intersection(result: &HSegmentIntersection) {
    match result {
        HSegmentIntersection::LineLine(result) => h_validate_line_line(result),
        HSegmentIntersection::LineArc { result, .. } => h_validate_line_arc(result),
        HSegmentIntersection::ArcArc(result) => h_validate_arc_arc(result),
    }
}

fn h_assert_line_arc_point_equivalent(
    left: &HLineArcIntersectionPoint,
    right: &HLineArcIntersectionPoint,
    context: &str,
) {
    h_assert_point_approx_eq(&left.point, &right.point, context);
    h_assert_scalar_approx_eq(&left.line_param, &right.line_param, context);
    assert_eq!(left.kind, right.kind, "{context}");
}

fn h_assert_arc_arc_point_equivalent(
    left: &HArcArcIntersectionPoint,
    right: &HArcArcIntersectionPoint,
    context: &str,
) {
    h_assert_point_approx_eq(&left.point, &right.point, context);
    assert_eq!(left.kind, right.kind, "{context}");
}

fn h_assert_line_line_intersection_equivalent(
    left: &HLineLineIntersection,
    right: &HLineLineIntersection,
    context: &str,
) {
    match (left, right) {
        (HLineLineIntersection::None, HLineLineIntersection::None) => {}
        (
            HLineLineIntersection::Point {
                point: left_point,
                a_param: left_a,
                b_param: left_b,
                kind: left_kind,
            },
            HLineLineIntersection::Point {
                point: right_point,
                a_param: right_a,
                b_param: right_b,
                kind: right_kind,
            },
        ) => {
            h_assert_point_approx_eq(left_point, right_point, context);
            h_assert_scalar_approx_eq(left_a, right_a, context);
            h_assert_scalar_approx_eq(left_b, right_b, context);
            assert_eq!(left_kind, right_kind, "{context}");
        }
        (
            HLineLineIntersection::Overlap {
                segment: left_segment,
                a_range: left_a,
                b_range: left_b,
            },
            HLineLineIntersection::Overlap {
                segment: right_segment,
                a_range: right_a,
                b_range: right_b,
            },
        ) => {
            h_assert_line_segment_approx_eq(left_segment, right_segment, context);
            h_assert_param_range_approx_eq(left_a, right_a, context);
            h_assert_param_range_approx_eq(left_b, right_b, context);
        }
        (
            HLineLineIntersection::Uncertain {
                reason: left_reason,
            },
            HLineLineIntersection::Uncertain {
                reason: right_reason,
            },
        ) => assert_eq!(left_reason, right_reason, "{context}"),
        _ => panic!(
            "{context}: expected equivalent line-line intersections, left={left:?}, right={right:?}"
        ),
    }
}

fn h_assert_line_arc_intersection_equivalent(
    left: &HLineArcIntersection,
    right: &HLineArcIntersection,
    context: &str,
) {
    match (left, right) {
        (HLineArcIntersection::None, HLineArcIntersection::None) => {}
        (HLineArcIntersection::Point(left), HLineArcIntersection::Point(right)) => {
            h_assert_line_arc_point_equivalent(left, right, context);
        }
        (
            HLineArcIntersection::TwoPoints {
                first: left_first,
                second: left_second,
            },
            HLineArcIntersection::TwoPoints {
                first: right_first,
                second: right_second,
            },
        ) => {
            h_assert_line_arc_point_equivalent(left_first, right_first, context);
            h_assert_line_arc_point_equivalent(left_second, right_second, context);
        }
        (
            HLineArcIntersection::Uncertain {
                reason: left_reason,
            },
            HLineArcIntersection::Uncertain {
                reason: right_reason,
            },
        ) => assert_eq!(left_reason, right_reason, "{context}"),
        _ => panic!(
            "{context}: expected equivalent line-arc intersections, left={left:?}, right={right:?}"
        ),
    }
}

fn h_assert_arc_arc_intersection_equivalent(
    left: &HArcArcIntersection,
    right: &HArcArcIntersection,
    context: &str,
) {
    match (left, right) {
        (HArcArcIntersection::None, HArcArcIntersection::None) => {}
        (HArcArcIntersection::Point(left), HArcArcIntersection::Point(right)) => {
            h_assert_arc_arc_point_equivalent(left, right, context);
        }
        (
            HArcArcIntersection::TwoPoints {
                first: left_first,
                second: left_second,
            },
            HArcArcIntersection::TwoPoints {
                first: right_first,
                second: right_second,
            },
        ) => {
            h_assert_arc_arc_point_equivalent(left_first, right_first, context);
            h_assert_arc_arc_point_equivalent(left_second, right_second, context);
        }
        (
            HArcArcIntersection::Overlap {
                segment: left_segment,
                a_range: left_a,
                b_range: left_b,
            },
            HArcArcIntersection::Overlap {
                segment: right_segment,
                a_range: right_a,
                b_range: right_b,
            },
        ) => {
            h_assert_arc_segment_approx_eq(left_segment, right_segment, context);
            h_assert_param_range_approx_eq(left_a, right_a, context);
            h_assert_param_range_approx_eq(left_b, right_b, context);
        }
        (
            HArcArcIntersection::Uncertain {
                reason: left_reason,
            },
            HArcArcIntersection::Uncertain {
                reason: right_reason,
            },
        ) => assert_eq!(left_reason, right_reason, "{context}"),
        _ => panic!(
            "{context}: expected equivalent arc-arc intersections, left={left:?}, right={right:?}"
        ),
    }
}

fn h_assert_segment_intersection_equivalent(
    left: &HSegmentIntersection,
    right: &HSegmentIntersection,
    context: &str,
) {
    match (left, right) {
        (HSegmentIntersection::LineLine(left), HSegmentIntersection::LineLine(right)) => {
            h_assert_line_line_intersection_equivalent(left, right, context);
        }
        (
            HSegmentIntersection::LineArc {
                order: left_order,
                result: left_result,
            },
            HSegmentIntersection::LineArc {
                order: right_order,
                result: right_result,
            },
        ) => {
            assert_eq!(left_order, right_order, "{context}");
            h_assert_line_arc_intersection_equivalent(left_result, right_result, context);
        }
        (HSegmentIntersection::ArcArc(left), HSegmentIntersection::ArcArc(right)) => {
            h_assert_arc_arc_intersection_equivalent(left, right, context);
        }
        _ => panic!(
            "{context}: expected equivalent segment intersections, left={left:?}, right={right:?}"
        ),
    }
}

fn h_assert_curve_string_intersections_equivalent(
    left: &[HCurveStringIntersection],
    right: &[HCurveStringIntersection],
) {
    assert_eq!(
        left.len(),
        right.len(),
        "curve-string intersection event counts should match"
    );
    for (left, right) in left.iter().zip(right) {
        assert_eq!(
            left.a_segment_index, right.a_segment_index,
            "curve-string first operand segment index should match"
        );
        assert_eq!(
            left.b_segment_index, right.b_segment_index,
            "curve-string second operand segment index should match"
        );
        h_assert_segment_intersection_equivalent(
            &left.relation,
            &right.relation,
            "curve-string relation",
        );
    }
}

/// Fuzz native line and circular-arc intersection dispatch.
pub fn h_assert_segment_intersections(reader: &mut ByteReader<'_>) {
    let first = h_segment_from_bytes(reader);
    let second = match reader.byte() % 4 {
        0 => h_segment_from_bytes(reader),
        1 => h_random_line(reader),
        2 => h_semicircle(reader),
        _ => HSegment2::Line(h_line_from_i32((-64, 0), (64, 0))),
    };
    let policy = h_policy();
    let forward = first.intersect_segment(&second, &policy).unwrap();
    let reverse = second.intersect_segment(&first, &policy).unwrap();

    assert_eq!(
        forward.is_none(),
        reverse.is_none(),
        "segment intersection none-ness should be symmetric"
    );
    h_validate_segment_intersection(&forward);
    h_validate_segment_intersection(&reverse);
}

/// Fuzz contour and region point classification, including explicit boundaries and holes.
pub fn h_assert_contour_region_classification(reader: &mut ByteReader<'_>) {
    let outer = h_rect_from_bytes(reader);
    let margin_x = reader.f64_range(outer.width() * 0.15, outer.width() * 0.35);
    let margin_y = reader.f64_range(outer.height() * 0.15, outer.height() * 0.35);
    let hole = outer.inset(margin_x, margin_y);
    let contour = h_rectangle_contour(outer);
    let region = HRegion2::new(vec![contour.clone()], vec![h_rectangle_contour(hole)]);
    let prepared = region.prepare_topology_queries(&h_policy());

    let samples = [
        (
            (outer.xmin + outer.xmax) * 0.5,
            outer.ymin + outer.height() * 0.05,
            HRegionPointLocation::Inside,
        ),
        (
            (hole.xmin + hole.xmax) * 0.5,
            (hole.ymin + hole.ymax) * 0.5,
            HRegionPointLocation::Outside,
        ),
        (
            outer.xmax + outer.width().max(1.0),
            outer.ymax + outer.height().max(1.0),
            HRegionPointLocation::Outside,
        ),
        (
            outer.xmin,
            (outer.ymin + outer.ymax) * 0.5,
            HRegionPointLocation::Boundary,
        ),
    ];

    for (x, y, expected) in samples {
        let point = h_point(x, y);
        assert_eq!(
            region.classify_point(&point, &h_policy()),
            HClassification::Decided(expected)
        );
        assert_eq!(
            prepared.classify_point(&point, &h_policy()),
            HClassification::Decided(expected)
        );
    }

    let area_report = match region.filled_area_report(&h_policy()).unwrap() {
        HClassification::Decided(report) => report,
        HClassification::Uncertain(reason) => {
            panic!("rectangle region area report should be decided: {reason:?}")
        }
    };
    assert!(area_report.is_complete());
    assert_eq!(
        HClassification::Decided(area_report.filled_area.clone()),
        region.filled_area(&h_policy()).unwrap()
    );
    assert_eq!(area_report.material_contour_count, 1);
    assert_eq!(area_report.hole_contour_count, 1);
    assert!(area_report.unsupported_contours.is_empty());

    assert_eq!(
        contour.classify_point(
            &h_point(
                (outer.xmin + outer.xmax) * 0.5,
                (outer.ymin + outer.ymax) * 0.5
            ),
            &h_policy(),
        ),
        HClassification::Decided(HContourPointLocation::Inside)
    );

    let projection_options = hypercurve::FiniteProjectionOptions::try_new(0.01).unwrap();
    let ring = contour.project_to_finite_ring(&projection_options).unwrap();
    assert!(ring.is_closed());
    assert_eq!(ring.certificate().source_segment_count(), contour.len());
    assert_eq!(
        ring.certificate().emitted_point_count(),
        ring.points().len()
    );
    assert_eq!(ring.certificate().line_segment_count(), contour.len());
    assert_eq!(ring.certificate().arc_segment_count(), 0);
    assert_eq!(ring.certificate().emitted_arc_sample_count(), 0);
    assert!(ring.points().first() == ring.points().last());

    let projected_region = region
        .project_to_finite_region(&projection_options)
        .unwrap();
    assert_eq!(projected_region.material_rings().len(), 1);
    assert_eq!(projected_region.hole_rings().len(), 1);
    assert_eq!(projected_region.certificate().material_ring_count(), 1);
    assert_eq!(projected_region.certificate().hole_ring_count(), 1);
    assert_eq!(projected_region.certificate().source_segment_count(), 8);
    assert_eq!(projected_region.certificate().line_segment_count(), 8);
    assert_eq!(projected_region.certificate().arc_segment_count(), 0);
    assert_eq!(
        projected_region.certificate().emitted_point_count(),
        projected_region
            .material_rings()
            .iter()
            .chain(projected_region.hole_rings())
            .map(|ring| ring.points().len())
            .sum::<usize>()
    );

    let imported = HCurveString2::import_finite_line_string(&[
        [outer.xmin, outer.ymin],
        [outer.xmax, outer.ymin],
        [outer.xmax, outer.ymin],
        [outer.xmax, outer.ymax],
    ])
    .unwrap();
    assert_eq!(imported.curve_string().len(), 2);
    assert_eq!(imported.certificate().input_point_count(), 4);
    assert_eq!(imported.certificate().skipped_duplicate_edge_count(), 1);
    assert_eq!(imported.certificate().output_segment_count(), 2);

    let imported_ring = HContour2::import_finite_ring(&[
        [outer.xmin, outer.ymin],
        [outer.xmax, outer.ymin],
        [outer.xmax, outer.ymax],
        [outer.xmin, outer.ymax],
        [outer.xmin, outer.ymin],
    ])
    .unwrap();
    assert_eq!(imported_ring.contour().len(), 4);
    assert!(imported_ring.certificate().repeated_closing_point());
    assert!(imported_ring.certificate().is_closed());
}

/// Fuzz native region booleans and prepared/ordinary consistency.
///
/// The boolean path follows Yap's exact-predicate discipline by checking both
/// exact and prepared dispatches side-by-side and only accepting reported
/// invariants when they remain decided under the active certified policy.
/// Specifically, degeneracy behavior follows boundary-contact handling from
/// Foster, Hormann, and Popa ("Clipping simple polygons with degenerate
/// intersections," *Computers & Graphics: X* 2, 100007, 2019), ensuring that
/// shared-boundary contacts are explicit blockers while unsupported traversal
/// cases remain explicit `UncertaintyReason::Unsupported`.
pub fn h_assert_region_boolean(reader: &mut ByteReader<'_>) {
    let a = h_region_from_bytes(reader, 3, 2);
    let b = h_region_from_bytes(reader, 3, 2);
    let op = h_boolean_op(reader);
    let policy = h_policy();
    let fill_rule = HFillRule::NonZero;
    let prepared_a = a.prepare_topology_queries(&policy);
    let prepared_b = b.prepare_topology_queries(&policy);

    let plain = a.boolean_region(&b, op, fill_rule, &policy).unwrap();
    assert_eq!(
        prepared_a
            .boolean_region(&prepared_b, op, fill_rule, &policy)
            .unwrap(),
        plain
    );
    assert_eq!(
        prepared_a
            .boolean_region_against_region(&b.as_view(), op, fill_rule, &policy)
            .unwrap(),
        plain
    );
    assert_eq!(
        a.as_view()
            .boolean_region_against_prepared_region(&prepared_b, op, fill_rule, &policy)
            .unwrap(),
        plain
    );

    let plain_contours = a
        .boolean_boundary_contours(&b, op, fill_rule, &policy)
        .unwrap();
    assert_eq!(
        prepared_a
            .boolean_boundary_contours(&prepared_b, op, fill_rule, &policy)
            .unwrap(),
        plain_contours
    );

    let plain_loops = a.boolean_boundary_loops(&b, op, &policy).unwrap();
    assert_eq!(
        prepared_a
            .boolean_boundary_loops(&prepared_b, op, &policy)
            .unwrap(),
        plain_loops
    );
    assert_eq!(
        prepared_a
            .boolean_boundary_loops_against_region(&b.as_view(), op, &policy)
            .unwrap(),
        plain_loops
    );
    assert_eq!(
        a.as_view()
            .boolean_boundary_loops_against_prepared_region(&prepared_b, op, &policy)
            .unwrap(),
        plain_loops
    );

    let shared_edge_a = HRegion2::from_material_contours(vec![h_rectangle_contour(HRect {
        xmin: 0.0,
        ymin: 0.0,
        xmax: 4.0,
        ymax: 4.0,
    })]);
    let shared_edge_b = HRegion2::from_material_contours(vec![h_rectangle_contour(HRect {
        xmin: 2.0,
        ymin: -2.0,
        xmax: 6.0,
        ymax: 0.0,
    })]);
    let point_touch_a = HRegion2::from_material_contours(vec![h_rectangle_contour(HRect {
        xmin: 0.0,
        ymin: 0.0,
        xmax: 2.0,
        ymax: 2.0,
    })]);
    let point_touch_b = HRegion2::from_material_contours(vec![h_rectangle_contour(HRect {
        xmin: 2.0,
        ymin: 2.0,
        xmax: 4.0,
        ymax: 4.0,
    })]);
    let shared_edge_prepared_a = shared_edge_a.prepare_topology_queries(&policy);
    let shared_edge_prepared_b = shared_edge_b.prepare_topology_queries(&policy);
    let point_touch_prepared_a = point_touch_a.prepare_topology_queries(&policy);
    let point_touch_prepared_b = point_touch_b.prepare_topology_queries(&policy);

    let shared_edge_plain_loops = shared_edge_a
        .boolean_boundary_loops(&shared_edge_b, HBooleanOp::Union, &policy)
        .unwrap();
    let shared_edge_prepared_loops = shared_edge_prepared_a
        .boolean_boundary_loops(&shared_edge_prepared_b, HBooleanOp::Union, &policy)
        .unwrap();
    let shared_edge_plain_contours = shared_edge_a
        .boolean_boundary_contours(
            &shared_edge_b,
            HBooleanOp::Union,
            HFillRule::NonZero,
            &policy,
        )
        .unwrap();
    let point_touch_plain_loops = point_touch_a
        .boolean_boundary_loops(&point_touch_b, HBooleanOp::Union, &policy)
        .unwrap();
    let point_touch_prepared_loops = point_touch_prepared_a
        .boolean_boundary_loops(&point_touch_prepared_b, HBooleanOp::Union, &policy)
        .unwrap();
    let point_touch_plain_contours = point_touch_a
        .boolean_boundary_contours(
            &point_touch_b,
            HBooleanOp::Union,
            HFillRule::NonZero,
            &policy,
        )
        .unwrap();

    assert_eq!(shared_edge_prepared_loops, shared_edge_plain_loops);
    assert_eq!(point_touch_prepared_loops, point_touch_plain_loops);
    let HClassification::Decided(shared_edge_loops) = &shared_edge_plain_loops else {
        panic!("shared-edge fuzz probe should regularize to decided boundary loops");
    };
    let HClassification::Decided(shared_edge_contours) = &shared_edge_plain_contours else {
        panic!("shared-edge fuzz probe should regularize to decided boundary contours");
    };
    assert_eq!(shared_edge_loops.len(), 1);
    assert_eq!(shared_edge_loops.len(), shared_edge_contours.len());
    let HClassification::Decided(point_touch_loops) = &point_touch_plain_loops else {
        panic!("point-touch fuzz probe should regularize to decided boundary loops");
    };
    let HClassification::Decided(point_touch_contours) = &point_touch_plain_contours else {
        panic!("point-touch fuzz probe should regularize to decided boundary contours");
    };
    assert_eq!(point_touch_loops.len(), 2);
    assert_eq!(point_touch_loops.len(), point_touch_contours.len());
    for contour in shared_edge_loops.to_contours(HFillRule::NonZero).unwrap() {
        h_assert_contour_finite(&contour);
    }

    let shared_edge_traversal = match shared_edge_a
        .boolean_boundary_traversal_report(&shared_edge_b, HBooleanOp::Union, &policy)
        .unwrap()
    {
        HClassification::Decided(report) => report,
        HClassification::Uncertain(reason) => {
            panic!("shared-edge fuzz traversal report should be decided: {reason:?}")
        }
    };
    assert_eq!(
        shared_edge_traversal.status,
        HBooleanBoundaryTraversalStatus::UnresolvedBoundaries
    );
    assert_eq!(
        shared_edge_traversal.blocker_reason,
        Some(HUncertaintyReason::Boundary),
    );

    let shared_edge_prepared_traversal = match shared_edge_prepared_a
        .boolean_boundary_traversal_report(&shared_edge_prepared_b, HBooleanOp::Union, &policy)
        .unwrap()
    {
        HClassification::Decided(report) => report,
        HClassification::Uncertain(reason) => {
            panic!("shared-edge prepared traversal report should be decided: {reason:?}")
        }
    };
    assert_eq!(shared_edge_prepared_traversal, shared_edge_traversal);
    let shared_edge_prepared_against_plain_traversal = match shared_edge_prepared_a
        .boolean_boundary_traversal_report_against_region(
            &shared_edge_b.as_view(),
            HBooleanOp::Union,
            &policy,
        )
        .unwrap()
    {
        HClassification::Decided(report) => report,
        HClassification::Uncertain(reason) => {
            panic!("shared-edge prepared-vs-plain traversal report should be decided: {reason:?}")
        }
    };
    let shared_edge_plain_against_prepared_traversal = match shared_edge_a
        .as_view()
        .boolean_boundary_traversal_report_against_prepared_region(
            &shared_edge_prepared_b,
            HBooleanOp::Union,
            &policy,
        )
        .unwrap()
    {
        HClassification::Decided(report) => report,
        HClassification::Uncertain(reason) => {
            panic!("shared-edge plain-vs-prepared traversal report should be decided: {reason:?}")
        }
    };

    // Point touch should be reported as an unsupported traversal blocker in this
    // raw traversal surface.
    let point_touch_traversal = match point_touch_a
        .boolean_boundary_traversal_report(&point_touch_b, HBooleanOp::Union, &policy)
        .unwrap()
    {
        HClassification::Decided(report) => report,
        HClassification::Uncertain(reason) => {
            panic!("point-touch fuzz traversal report should be decided: {reason:?}")
        }
    };
    assert_eq!(
        point_touch_traversal.status,
        HBooleanBoundaryTraversalStatus::UnsupportedTraversal
    );
    assert_eq!(
        point_touch_traversal.blocker_reason,
        Some(HUncertaintyReason::Unsupported)
    );

    let point_touch_prepared_traversal = match point_touch_prepared_a
        .boolean_boundary_traversal_report(&point_touch_prepared_b, HBooleanOp::Union, &policy)
        .unwrap()
    {
        HClassification::Decided(report) => report,
        HClassification::Uncertain(reason) => {
            panic!("point-touch prepared traversal report should be decided: {reason:?}")
        }
    };
    assert_eq!(point_touch_prepared_traversal, point_touch_traversal);
    let point_touch_prepared_against_plain = match point_touch_prepared_a
        .boolean_boundary_traversal_report_against_region(
            &point_touch_b.as_view(),
            HBooleanOp::Union,
            &policy,
        )
        .unwrap()
    {
        HClassification::Decided(report) => report,
        HClassification::Uncertain(reason) => {
            panic!("point-touch prepared-vs-plain traversal report should be decided: {reason:?}")
        }
    };
    let point_touch_plain_against_prepared = match point_touch_a
        .as_view()
        .boolean_boundary_traversal_report_against_prepared_region(
            &point_touch_prepared_b,
            HBooleanOp::Union,
            &policy,
        )
        .unwrap()
    {
        HClassification::Decided(report) => report,
        HClassification::Uncertain(reason) => {
            panic!("point-touch plain-vs-prepared traversal report should be decided: {reason:?}")
        }
    };

    assert_eq!(
        shared_edge_prepared_traversal,
        shared_edge_prepared_against_plain_traversal
    );
    assert_eq!(
        shared_edge_plain_against_prepared_traversal,
        shared_edge_prepared_traversal
    );
    assert_eq!(
        point_touch_prepared_traversal,
        point_touch_prepared_against_plain
    );
    assert_eq!(
        point_touch_plain_against_prepared,
        point_touch_prepared_traversal
    );
    for contour in point_touch_loops.to_contours(HFillRule::NonZero).unwrap() {
        h_assert_contour_finite(&contour);
    }

    // Foster, Hormann, and Popa (2019) require full shared-edge contacts inside
    // holes to be regularized consistently before traversal as a canonical
    // contour product instead of unresolved boundary-only blockers.
    let hole_with_cavity = HRegion2::new(
        vec![h_rectangle_contour(HRect {
            xmin: 0.0,
            ymin: 0.0,
            xmax: 10.0,
            ymax: 10.0,
        })],
        vec![h_rectangle_contour(HRect {
            xmin: 3.0,
            ymin: 3.0,
            xmax: 7.0,
            ymax: 7.0,
        })],
    );
    let hole_strip = HRegion2::from_material_contours(vec![h_rectangle_contour(HRect {
        xmin: 4.0,
        ymin: 3.0,
        xmax: 6.0,
        ymax: 5.0,
    })]);
    let prepared_hole_with_cavity = hole_with_cavity.prepare_topology_queries(&policy);
    let prepared_hole_strip = hole_strip.prepare_topology_queries(&policy);

    let HClassification::Decided(hole_strip_loops) = hole_with_cavity
        .boolean_boundary_loops(&hole_strip, HBooleanOp::Union, &policy)
        .unwrap()
    else {
        panic!("hole-boundary touching strip union should be decided");
    };
    assert_eq!(
        prepared_hole_with_cavity
            .boolean_boundary_loops(&prepared_hole_strip, HBooleanOp::Union, &policy)
            .unwrap(),
        HClassification::Decided(hole_strip_loops.clone())
    );
    assert_eq!(
        prepared_hole_with_cavity
            .boolean_boundary_loops_against_region(
                &hole_strip.as_view(),
                HBooleanOp::Union,
                &policy,
            )
            .unwrap(),
        HClassification::Decided(hole_strip_loops.clone())
    );
    assert_eq!(
        hole_with_cavity
            .as_view()
            .boolean_boundary_loops_against_prepared_region(
                &prepared_hole_strip,
                HBooleanOp::Union,
                &policy,
            )
            .unwrap(),
        HClassification::Decided(hole_strip_loops.clone())
    );
    let HClassification::Decided(hole_strip_contours) = hole_with_cavity
        .boolean_boundary_contours(&hole_strip, HBooleanOp::Union, HFillRule::NonZero, &policy)
        .unwrap()
    else {
        panic!("hole-boundary touching strip union contours should be decided");
    };
    assert_eq!(
        prepared_hole_with_cavity
            .boolean_boundary_contours(
                &prepared_hole_strip,
                HBooleanOp::Union,
                HFillRule::NonZero,
                &policy,
            )
            .unwrap(),
        HClassification::Decided(hole_strip_contours.clone())
    );
    for contour in hole_strip_loops.to_contours(HFillRule::NonZero).unwrap() {
        h_assert_contour_finite(&contour);
    }
    for contour in hole_strip_contours {
        h_assert_contour_finite(&contour);
    }

    let traversal_report = match a
        .boolean_boundary_traversal_report(&b, op, &policy)
        .unwrap()
    {
        HClassification::Decided(report) => report,
        HClassification::Uncertain(reason) => {
            panic!("plain traversal report should classify raw loop status: {reason:?}")
        }
    };
    assert_eq!(
        prepared_a
            .boolean_boundary_traversal_report(&prepared_b, op, &policy)
            .unwrap(),
        HClassification::Decided(traversal_report.clone())
    );
    assert_eq!(
        prepared_a
            .boolean_boundary_traversal_report_against_region(&b.as_view(), op, &policy)
            .unwrap(),
        HClassification::Decided(traversal_report.clone())
    );
    assert_eq!(
        a.as_view()
            .boolean_boundary_traversal_report_against_prepared_region(&prepared_b, op, &policy)
            .unwrap(),
        HClassification::Decided(traversal_report.clone())
    );
    assert_eq!(
        traversal_report.is_ready(),
        matches!(
            traversal_report.status,
            HBooleanBoundaryTraversalStatus::Empty | HBooleanBoundaryTraversalStatus::LoopsReady
        )
    );
    if let HClassification::Decided(loops) = &plain_loops {
        if traversal_report.is_ready() {
            assert_eq!(traversal_report.loops.as_ref(), Some(loops));
            assert_eq!(traversal_report.blocker_reason, None);
        }
    } else {
        assert!(!traversal_report.is_ready());
        assert!(traversal_report.blocker_reason.is_some());
    }

    if let HClassification::Decided(result) = &plain {
        let report = match a.boolean_region_report(&b, op, fill_rule, &policy).unwrap() {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(reason) => {
                panic!("plain boolean report should match decided boolean result: {reason:?}")
            }
        };
        assert_eq!(
            prepared_a
                .boolean_region_report(&prepared_b, op, fill_rule, &policy)
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(
            prepared_a
                .boolean_region_report_against_region(&b.as_view(), op, fill_rule, &policy)
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(
            a.as_view()
                .boolean_region_report_against_prepared_region(&prepared_b, op, fill_rule, &policy,)
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(&report.result, result);
        assert_eq!(
            report.audit.is_valid(),
            matches!(
                report.audit.status,
                HBooleanRegionAuditStatus::Empty | HBooleanRegionAuditStatus::Valid
            )
        );
        for contour in result
            .material_contours()
            .iter()
            .chain(result.hole_contours().iter())
        {
            h_assert_contour_finite(contour);
        }
        h_assert_region_semantics(&a, &b, result, op);
    }

    if let (HClassification::Decided(result), HClassification::Decided(contours)) =
        (&plain, &plain_contours)
    {
        let report = match a
            .boolean_region_pipeline_report(&b, op, fill_rule, &policy)
            .unwrap()
        {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(reason) => {
                panic!("plain pipeline report should match decided region: {reason:?}")
            }
        };
        assert_eq!(
            prepared_a
                .boolean_region_pipeline_report(&prepared_b, op, fill_rule, &policy)
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(
            prepared_a
                .boolean_region_pipeline_report_against_region(
                    &b.as_view(),
                    op,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(
            a.as_view()
                .boolean_region_pipeline_report_against_prepared_region(
                    &prepared_b,
                    op,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(&report.result, result);
        assert_eq!(&report.boundary_contours, contours);
        assert_eq!(report.boundary_audit.contour_count, contours.len());
        assert_eq!(report.nesting_audit.input_contour_count, contours.len());
        assert_eq!(
            report.nesting_audit.material_contour_count + report.nesting_audit.hole_contour_count,
            contours.len()
        );
        assert_eq!(
            report.region_audit.is_valid(),
            matches!(
                report.region_audit.status,
                HBooleanRegionAuditStatus::Empty | HBooleanRegionAuditStatus::Valid
            )
        );
    }

    if let HClassification::Decided(contours) = plain_contours {
        let report = match a
            .boolean_boundary_contour_report(&b, op, fill_rule, &policy)
            .unwrap()
        {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(reason) => {
                panic!("plain boundary report should match decided contours: {reason:?}")
            }
        };
        assert_eq!(
            prepared_a
                .boolean_boundary_contour_report(&prepared_b, op, fill_rule, &policy)
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(
            prepared_a
                .boolean_boundary_contour_report_against_region(
                    &b.as_view(),
                    op,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(
            a.as_view()
                .boolean_boundary_contour_report_against_prepared_region(
                    &prepared_b,
                    op,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(&report.contours, &contours);
        assert_eq!(
            report.audit.is_valid(),
            matches!(
                report.audit.status,
                HBooleanBoundaryAuditStatus::Empty | HBooleanBoundaryAuditStatus::Valid
            )
        );
        for contour in &contours {
            h_assert_contour_finite(contour);
        }
    }

    if let HClassification::Decided(loops) = plain_loops {
        let report = match a
            .boolean_boundary_loop_report(&b, op, fill_rule, &policy)
            .unwrap()
        {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(reason) => {
                panic!("plain boundary loop report should match decided loops: {reason:?}")
            }
        };
        assert_eq!(
            prepared_a
                .boolean_boundary_loop_report(&prepared_b, op, fill_rule, &policy)
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(
            prepared_a
                .boolean_boundary_loop_report_against_region(&b.as_view(), op, fill_rule, &policy,)
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(
            a.as_view()
                .boolean_boundary_loop_report_against_prepared_region(
                    &prepared_b,
                    op,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(&report.loops, &loops);
        assert_eq!(
            report.audit.is_valid(),
            matches!(
                report.audit.status,
                HBooleanBoundaryAuditStatus::Empty | HBooleanBoundaryAuditStatus::Valid
            )
        );
        for contour in loops.to_contours(fill_rule).unwrap() {
            h_assert_contour_finite(&contour);
        }
    }
}

/// Fuzz contract for adversarial degenerate boolean pairs.
///
/// This contract mirrors Yap's exactness contract (1997): every API flavor should
/// return equivalent decided artifacts when called through plain, prepared, or mixed
/// prepared/view call surfaces. It also keeps degeneracy blockers visible following
/// Foster, Hormann, and Popa's treatment of boundary intersections (2019).
pub fn h_assert_region_boolean_antagonistic_contract(reader: &mut ByteReader<'_>) {
    let policy = h_policy();
    let fill_rule = HFillRule::NonZero;
    let _ = reader.byte();

    let shared_edge_a = HRegion2::from_material_contours(vec![h_rectangle_contour(HRect {
        xmin: 0.0,
        ymin: 0.0,
        xmax: 4.0,
        ymax: 4.0,
    })]);
    let shared_edge_b = HRegion2::from_material_contours(vec![h_rectangle_contour(HRect {
        xmin: 2.0,
        ymin: -2.0,
        xmax: 6.0,
        ymax: 0.0,
    })]);
    let point_touch_a = HRegion2::from_material_contours(vec![h_rectangle_contour(HRect {
        xmin: 0.0,
        ymin: 0.0,
        xmax: 2.0,
        ymax: 2.0,
    })]);
    let point_touch_b = HRegion2::from_material_contours(vec![h_rectangle_contour(HRect {
        xmin: 2.0,
        ymin: 2.0,
        xmax: 4.0,
        ymax: 4.0,
    })]);
    let hole_outer = HRegion2::new(
        vec![h_rectangle_contour(HRect {
            xmin: 0.0,
            ymin: 0.0,
            xmax: 12.0,
            ymax: 12.0,
        })],
        vec![h_rectangle_contour(HRect {
            xmin: 4.0,
            ymin: 4.0,
            xmax: 8.0,
            ymax: 8.0,
        })],
    );
    let hole_strip = HRegion2::from_material_contours(vec![h_rectangle_contour(HRect {
        xmin: 6.0,
        ymin: 2.0,
        xmax: 10.0,
        ymax: 10.0,
    })]);

    let cases: [(
        HRegion2,
        HRegion2,
        HBooleanBoundaryTraversalStatus,
        Option<HUncertaintyReason>,
    ); 3] = [
        (
            shared_edge_a,
            shared_edge_b,
            HBooleanBoundaryTraversalStatus::UnresolvedBoundaries,
            Some(HUncertaintyReason::Boundary),
        ),
        (
            point_touch_a,
            point_touch_b,
            HBooleanBoundaryTraversalStatus::UnsupportedTraversal,
            Some(HUncertaintyReason::Unsupported),
        ),
        (
            hole_outer,
            hole_strip,
            HBooleanBoundaryTraversalStatus::LoopsReady,
            None,
        ),
    ];

    for (left, right, expected_status, expected_blocker) in cases {
        let left_prepared = left.prepare_topology_queries(&policy);
        let right_prepared = right.prepare_topology_queries(&policy);

        let traversal = match left
            .boolean_boundary_traversal_report(&right, HBooleanOp::Union, &policy)
            .unwrap()
        {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(reason) => {
                panic!("antagonistic contract traversal should be decided: {reason:?}")
            }
        };
        h_assert_traversal_report_invariants(&traversal, expected_status, expected_blocker);
        assert_eq!(
            left_prepared
                .boolean_boundary_traversal_report(&right_prepared, HBooleanOp::Union, &policy)
                .unwrap(),
            HClassification::Decided(traversal.clone())
        );
        assert_eq!(
            left_prepared
                .boolean_boundary_traversal_report_against_region(
                    &right.as_view(),
                    HBooleanOp::Union,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(traversal.clone())
        );
        assert_eq!(
            left.as_view()
                .boolean_boundary_traversal_report_against_prepared_region(
                    &right_prepared,
                    HBooleanOp::Union,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(traversal)
        );

        let region = match left
            .boolean_region(&right, HBooleanOp::Union, fill_rule, &policy)
            .unwrap()
        {
            HClassification::Decided(region) => region,
            HClassification::Uncertain(reason) => {
                panic!("antagonistic contract plain region boolean should be decided: {reason:?}")
            }
        };
        let boundary_contour_report = match left
            .boolean_boundary_contour_report(&right, HBooleanOp::Union, fill_rule, &policy)
            .unwrap()
        {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(reason) => {
                panic!(
                    "antagonistic contract boundary contour report should be decided: {reason:?}"
                )
            }
        };
        let boundary_loop_report = match left
            .boolean_boundary_loop_report(&right, HBooleanOp::Union, fill_rule, &policy)
            .unwrap()
        {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(reason) => {
                panic!("antagonistic contract boundary loop report should be decided: {reason:?}")
            }
        };
        let region_report = match left
            .boolean_region_report(&right, HBooleanOp::Union, fill_rule, &policy)
            .unwrap()
        {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(reason) => {
                panic!("antagonistic contract region report should be decided: {reason:?}")
            }
        };
        let pipeline_report = match left
            .boolean_region_pipeline_report(&right, HBooleanOp::Union, fill_rule, &policy)
            .unwrap()
        {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(reason) => {
                panic!("antagonistic contract pipeline report should be decided: {reason:?}")
            }
        };

        assert_eq!(boundary_loop_report.operation, HBooleanOp::Union);
        assert_eq!(boundary_contour_report.operation, HBooleanOp::Union);
        assert_eq!(region_report.operation, HBooleanOp::Union);
        assert_eq!(pipeline_report.operation, HBooleanOp::Union);

        assert_eq!(&region_report.result, &region);
        assert_eq!(&pipeline_report.result, &region);
        assert_eq!(
            &pipeline_report.boundary_contours,
            &boundary_contour_report.contours
        );
        assert_eq!(
            boundary_loop_report.loops.len(),
            boundary_contour_report.contours.len()
        );
        h_assert_boundary_audit_is_valid(&boundary_contour_report.audit.status);
        h_assert_boundary_audit_is_valid(&boundary_loop_report.audit.status);
        h_assert_region_audit_is_valid(&region_report.audit.status);
        h_assert_boundary_audit_is_valid(&pipeline_report.boundary_audit.status);
        assert!(pipeline_report.nesting_audit.is_valid());
        h_assert_region_audit_is_valid(&pipeline_report.region_audit.status);
        assert_eq!(
            pipeline_report.boundary_audit.contour_count,
            boundary_contour_report.contours.len()
        );
        assert_eq!(
            pipeline_report.nesting_audit.material_contour_count
                + pipeline_report.nesting_audit.hole_contour_count,
            boundary_contour_report.contours.len()
        );

        assert_eq!(
            left_prepared
                .boolean_boundary_contour_report(
                    &right_prepared,
                    HBooleanOp::Union,
                    fill_rule,
                    &policy
                )
                .unwrap(),
            HClassification::Decided(boundary_contour_report.clone())
        );
        assert_eq!(
            left_prepared
                .boolean_boundary_contour_report_against_region(
                    &right.as_view(),
                    HBooleanOp::Union,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(boundary_contour_report.clone())
        );
        assert_eq!(
            left.as_view()
                .boolean_boundary_contour_report_against_prepared_region(
                    &right_prepared,
                    HBooleanOp::Union,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(boundary_contour_report.clone())
        );
        assert_eq!(
            left_prepared
                .boolean_boundary_loop_report(
                    &right_prepared,
                    HBooleanOp::Union,
                    fill_rule,
                    &policy
                )
                .unwrap(),
            HClassification::Decided(boundary_loop_report.clone())
        );
        assert_eq!(
            left_prepared
                .boolean_boundary_loop_report_against_region(
                    &right.as_view(),
                    HBooleanOp::Union,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(boundary_loop_report.clone())
        );
        assert_eq!(
            left.as_view()
                .boolean_boundary_loop_report_against_prepared_region(
                    &right_prepared,
                    HBooleanOp::Union,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(boundary_loop_report.clone())
        );

        assert_eq!(
            left_prepared
                .boolean_region_report(&right_prepared, HBooleanOp::Union, fill_rule, &policy)
                .unwrap(),
            HClassification::Decided(region_report.clone())
        );
        assert_eq!(
            left_prepared
                .boolean_region_report_against_region(
                    &right.as_view(),
                    HBooleanOp::Union,
                    fill_rule,
                    &policy
                )
                .unwrap(),
            HClassification::Decided(region_report.clone())
        );
        assert_eq!(
            left.as_view()
                .boolean_region_report_against_prepared_region(
                    &right_prepared,
                    HBooleanOp::Union,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(region_report.clone())
        );
        assert_eq!(
            left_prepared
                .boolean_region_pipeline_report(
                    &right_prepared,
                    HBooleanOp::Union,
                    fill_rule,
                    &policy
                )
                .unwrap(),
            HClassification::Decided(pipeline_report.clone())
        );
        assert_eq!(
            left_prepared
                .boolean_region_pipeline_report_against_region(
                    &right.as_view(),
                    HBooleanOp::Union,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(pipeline_report.clone())
        );
        assert_eq!(
            left.as_view()
                .boolean_region_pipeline_report_against_prepared_region(
                    &right_prepared,
                    HBooleanOp::Union,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(pipeline_report.clone())
        );
    }
}

/// Fuzz contour/region event collection and split-map consistency.
pub fn h_assert_events_and_fragments(reader: &mut ByteReader<'_>) {
    let a = h_rectangle_contour(h_rect_from_bytes(reader));
    let b = h_rectangle_contour(h_rect_from_bytes(reader));
    let policy = h_policy();
    let prepared_a = a.prepare_topology_queries(&policy);
    let prepared_b = b.prepare_topology_queries(&policy);
    let events = a.intersect_contour(&b, &policy).unwrap();

    assert_eq!(
        prepared_a
            .intersect_prepared_contour(&prepared_b, &policy)
            .unwrap(),
        events
    );
    assert_eq!(prepared_a.intersect_contour(&b, &policy).unwrap(), events);

    for operand in [HContourOperand::First, HContourOperand::Second] {
        let source = if matches!(operand, HContourOperand::First) {
            &a
        } else {
            &b
        };
        if let HClassification::Decided(fragments) = source
            .split_at_intersections(&events, operand, &policy)
            .unwrap()
        {
            for fragment in fragments.fragments() {
                h_assert_segment_finite(&fragment.segment);
                h_assert_scalar_unit_interval(fragment.source_range.start());
                h_assert_scalar_unit_interval(fragment.source_range.end());
            }
        }
    }

    let region_a = HRegion2::new(vec![a], vec![]);
    let region_b = HRegion2::new(vec![b], vec![]);
    let region_events = region_a.intersect_region(&region_b, &policy).unwrap();
    let prepared_region_a = region_a.prepare_topology_queries(&policy);
    let prepared_region_b = region_b.prepare_topology_queries(&policy);
    assert_eq!(
        prepared_region_a
            .intersect_prepared_region(&prepared_region_b, &policy)
            .unwrap(),
        region_events
    );
}

fn h_curve_string_from_bytes(reader: &mut ByteReader<'_>) -> HCurveString2 {
    let count = reader.usize_range(2, 6);
    let mut x = reader.f64_range(-48.0, 48.0);
    let mut y = reader.f64_range(-48.0, 48.0);
    let mut vertices = Vec::with_capacity(count);
    for index in 0..count {
        let bulge = if index + 1 < count {
            match reader.byte() % 5 {
                0 => -1.0,
                1 => -0.5,
                2 => 0.0,
                3 => 0.5,
                _ => 1.0,
            }
        } else {
            0.0
        };
        vertices.push(h_vertex(x, y, bulge));
        if index + 1 < count {
            x += reader.f64_range(0.25, 24.0);
            y += reader.f64_range(-12.0, 12.0);
        }
    }
    HCurveString2::from_bulge_vertices(&vertices).unwrap()
}

fn h_assert_aabb_finite(bbox: &HAabb2) {
    h_assert_point_finite(bbox.min());
    h_assert_point_finite(bbox.max());
    let min_x = bbox.min_x().to_f64_approx().unwrap();
    let min_y = bbox.min_y().to_f64_approx().unwrap();
    let max_x = bbox.max_x().to_f64_approx().unwrap();
    let max_y = bbox.max_y().to_f64_approx().unwrap();
    assert!(min_x <= max_x, "aabb x coordinates are inverted");
    assert!(min_y <= max_y, "aabb y coordinates are inverted");
}

fn h_assert_aabb_contains_point(bbox: &HAabb2, point: &HPoint, policy: &HCurvePolicy) {
    assert_eq!(
        bbox.contains_point(point, policy),
        HClassification::Decided(true)
    );
}

fn h_assert_aabb_contains_segment_endpoints(
    bbox: &HAabb2,
    segment: &HSegment,
    policy: &HCurvePolicy,
) {
    match segment {
        HSegment2::Line(line) => {
            h_assert_aabb_contains_point(bbox, line.start(), policy);
            h_assert_aabb_contains_point(bbox, line.end(), policy);
        }
        HSegment2::Arc(arc) => {
            h_assert_aabb_contains_point(bbox, arc.start(), policy);
            h_assert_aabb_contains_point(bbox, arc.end(), policy);
        }
    }
}

fn h_validate_split_map(map: &HContourSplitMap) {
    assert_eq!(
        map.split_points().first().map(|point| point.segment_index),
        Some(0)
    );
    let mut previous_segment = 0;
    let mut previous_param = f64::NEG_INFINITY;

    for point in map.split_points() {
        h_assert_scalar_unit_interval(&point.param);
        let param = point.param.to_f64_approx().unwrap();
        assert!(
            point.segment_index >= previous_segment,
            "split points should be sorted by segment index"
        );
        if point.segment_index == previous_segment {
            assert!(
                param + 1e-8 >= previous_param,
                "split parameters should be sorted within a segment"
            );
        } else {
            previous_segment = point.segment_index;
        }
        previous_param = param;
    }

    for segment_index in 0..map.segment_count() {
        let params = map
            .params_for_segment(segment_index)
            .expect("split map should expose every source segment");
        assert_eq!(params.first(), Some(&h_scalar_i32(0)));
        assert_eq!(params.last(), Some(&h_scalar_i32(1)));
        let mut previous = f64::NEG_INFINITY;
        for param in params {
            h_assert_scalar_unit_interval(param);
            let value = param.to_f64_approx().unwrap();
            assert!(
                value + 1e-8 >= previous,
                "split map parameters should be sorted"
            );
            previous = value;
        }
    }
}

/// Fuzz bbox, prepared curve-string, and split-map invariants.
pub fn h_assert_bboxes_curve_strings_and_splits(reader: &mut ByteReader<'_>) {
    let policy = h_policy();
    let first = h_segment_from_bytes(reader);
    let second = h_segment_from_bytes(reader);

    let first_box = HAabb2::from_segment(&first, &policy).unwrap();
    let second_box = HAabb2::from_segment(&second, &policy).unwrap();
    if let HClassification::Decided(first_box) = &first_box {
        h_assert_aabb_finite(first_box);
        h_assert_aabb_contains_segment_endpoints(first_box, &first, &policy);
    }
    if let HClassification::Decided(second_box) = &second_box {
        h_assert_aabb_finite(second_box);
        h_assert_aabb_contains_segment_endpoints(second_box, &second, &policy);
    }
    if let (HClassification::Decided(first_box), HClassification::Decided(second_box)) =
        (&first_box, &second_box)
    {
        assert_eq!(
            first_box.overlaps(second_box, &policy),
            second_box.overlaps(first_box, &policy),
            "aabb overlap must be symmetric"
        );
        if let HClassification::Decided(union) = first_box.union(second_box, &policy) {
            h_assert_aabb_finite(&union);
            h_assert_aabb_contains_segment_endpoints(&union, &first, &policy);
            h_assert_aabb_contains_segment_endpoints(&union, &second, &policy);
        }
    }

    let curve_a = h_curve_string_from_bytes(reader);
    let curve_b = h_curve_string_from_bytes(reader);
    if let HClassification::Decided(curve_box) =
        HAabb2::from_curve_string(&curve_a, &policy).unwrap()
    {
        h_assert_aabb_finite(&curve_box);
        for segment in curve_a.segments() {
            h_assert_aabb_contains_segment_endpoints(&curve_box, segment, &policy);
        }
    }

    let prepared_a = curve_a.prepare_topology_queries(&policy);
    let prepared_b = curve_b.prepare_topology_queries(&policy);
    assert_eq!(prepared_a.curve_string(), &curve_a);
    assert_eq!(prepared_a.segment_boxes().len(), curve_a.segments().len());
    if let Some(curve_box) = prepared_a.curve_box() {
        h_assert_aabb_finite(curve_box);
    }
    for bbox in prepared_a.segment_boxes().iter().flatten() {
        h_assert_aabb_finite(bbox);
    }

    let plain_events = curve_a.intersect_curve_string(&curve_b, &policy).unwrap();
    h_assert_curve_string_intersections_equivalent(
        &prepared_a
            .intersect_prepared_curve_string(&prepared_b, &policy)
            .unwrap(),
        &plain_events,
    );
    h_assert_curve_string_intersections_equivalent(
        &prepared_a
            .intersect_curve_string(&curve_b, &policy)
            .unwrap(),
        &plain_events,
    );
    for event in &plain_events {
        h_validate_segment_intersection(&event.relation);
    }

    let contour_a = h_rectangle_contour(h_rect_from_bytes(reader));
    let contour_b = h_rectangle_contour(h_rect_from_bytes(reader));
    let contour_events = contour_a.intersect_contour(&contour_b, &policy).unwrap();
    for (operand, segment_count) in [
        (HContourOperand::First, contour_a.len()),
        (HContourOperand::Second, contour_b.len()),
    ] {
        if let HClassification::Decided(split_map) =
            HContourSplitMap::from_intersections(segment_count, &contour_events, operand, &policy)
        {
            h_validate_split_map(&split_map);
        }
    }
}

/// Fuzz primitive, curve-string, contour, and outline offsets.
pub fn h_assert_offsets_and_self_contacts(reader: &mut ByteReader<'_>) {
    let policy = h_offset_policy();
    let HSegment2::Line(line) = h_random_line(reader) else {
        unreachable!("h_random_line always returns a line segment");
    };
    let distance = h_scalar(reader.f64_range(-8.0, 8.0));
    if let HClassification::Decided(offset) = HSegment2::Line(line.clone())
        .offset_left(distance.clone(), &policy)
        .unwrap()
    {
        h_assert_segment_finite(&offset);
    }

    let curve = h_curve_string_from_bytes(reader);
    let _ = curve.has_self_contacts(&policy).unwrap();
    if let HClassification::Decided(offset) = curve
        .offset_left_with_line_joins(distance.clone(), &policy)
        .unwrap()
    {
        h_assert_curve_string_finite(&offset);
    }
    if let HClassification::Decided(offset) = curve
        .offset_left_checked(distance.clone(), &policy)
        .unwrap()
    {
        h_assert_curve_string_finite(&offset);
    }

    let outline_distance = h_scalar(reader.f64_range(0.01, 8.0));
    let cap = match reader.byte() % 3 {
        0 => HOffsetCap::Round,
        1 => HOffsetCap::Butt,
        _ => HOffsetCap::Square,
    };
    if let HClassification::Decided(outline) = curve
        .offset_outline(outline_distance.clone(), cap, &policy)
        .unwrap()
    {
        h_assert_contour_finite(&outline);
    }

    let rect = h_rectangle_contour(h_rect_from_bytes(reader));
    let _ = rect.has_self_contacts(&policy).unwrap();
    if let HClassification::Decided(offset) = rect
        .offset_left_with_line_joins(distance.clone(), &policy)
        .unwrap()
    {
        h_assert_contour_finite(&offset);
    }
    if let HClassification::Decided(offset) = rect.offset_left_checked(distance, &policy).unwrap() {
        h_assert_contour_finite(&offset);
    }
}

fn h_assert_segment_contains_core(segment: &HSegment, policy: &HCurvePolicy) {
    assert_eq!(
        segment.contains_point(segment.start(), policy),
        HClassification::Decided(true)
    );
    assert_eq!(
        segment.contains_point(segment.end(), policy),
        HClassification::Decided(true)
    );

    if let HClassification::Decided(representative) = segment.representative_point(policy).unwrap()
    {
        h_assert_point_finite(&representative);
        assert_eq!(
            segment.contains_point(&representative, policy),
            HClassification::Decided(true)
        );
        if let HClassification::Decided(bbox) = HAabb2::from_segment(segment, policy).unwrap() {
            h_assert_aabb_contains_point(&bbox, &representative, policy);
        }
    }
}

/// Fuzz primitive containment, reversal, representative-point, and bbox APIs.
pub fn h_assert_segment_containment_and_reversal(reader: &mut ByteReader<'_>) {
    let policy = h_policy();
    let segment = h_segment_from_bytes(reader);
    h_assert_segment_finite(&segment);
    h_assert_segment_contains_core(&segment, &policy);

    let reversed = segment.reversed();
    h_assert_segment_finite(&reversed);
    h_assert_segment_contains_core(&reversed, &policy);
    assert_eq!(reversed.reversed(), segment);
}

fn h_nested_rect_stack(reader: &mut ByteReader<'_>) -> Vec<HRect> {
    let depth = reader.usize_range(3, 6);
    let x = reader.f64_range(-48.0, 48.0);
    let y = reader.f64_range(-48.0, 48.0);
    let step = reader.f64_range(1.5, 8.0);
    let inner_width = reader.f64_range(2.0, 24.0);
    let inner_height = reader.f64_range(2.0, 24.0);
    let width = inner_width + 2.0 * step * (depth.saturating_sub(1) as f64);
    let height = inner_height + 2.0 * step * (depth.saturating_sub(1) as f64);

    (0..depth)
        .map(|index| {
            let inset = step * index as f64;
            HRect {
                xmin: x + inset,
                ymin: y + inset,
                xmax: x + width - inset,
                ymax: y + height - inset,
            }
        })
        .collect()
}

/// Fuzz contour nesting into material and hole bins, including prepared classifiers.
pub fn h_assert_boundary_nesting(reader: &mut ByteReader<'_>) {
    let policy = h_policy();
    let rects = h_nested_rect_stack(reader);
    let contours: Vec<_> = rects.iter().copied().map(h_rectangle_contour).collect();
    let region = match HRegion2::from_boundary_contours(contours, &policy).unwrap() {
        HClassification::Decided(region) => region,
        HClassification::Uncertain(_) => return,
    };
    let prepared = region.prepare_topology_queries(&policy);

    assert_eq!(region.material_contours().len(), (rects.len() + 1) / 2);
    assert_eq!(region.hole_contours().len(), rects.len() / 2);

    for (depth, rect) in rects.iter().enumerate() {
        let sample = h_point(
            rect.xmin + rect.width().min(1.0) * 0.25,
            rect.ymin + rect.height().min(1.0) * 0.25,
        );
        let expected = if depth % 2 == 0 {
            HRegionPointLocation::Inside
        } else {
            HRegionPointLocation::Outside
        };
        assert_eq!(
            region.classify_point(&sample, &policy),
            HClassification::Decided(expected)
        );
        assert_eq!(
            prepared.classify_point(&sample, &policy),
            HClassification::Decided(expected)
        );
    }
}

fn h_validate_region_fragments(fragments: &hypercurve::RegionFragmentSet, policy: &HCurvePolicy) {
    for contour_fragments in fragments.contours() {
        let mut previous_source = 0_usize;
        for (fragment_index, fragment) in contour_fragments.fragments.fragments().iter().enumerate()
        {
            h_assert_segment_finite(&fragment.segment);
            h_assert_scalar_unit_interval(fragment.source_range.start());
            h_assert_scalar_unit_interval(fragment.source_range.end());
            if fragment_index > 0 {
                assert!(
                    fragment.source_segment_index >= previous_source,
                    "region fragments should be emitted in source order"
                );
            }
            previous_source = fragment.source_segment_index;
            if let HClassification::Decided(sample) =
                fragment.segment.representative_point(policy).unwrap()
            {
                h_assert_point_finite(&sample);
            }
        }
    }
}

/// Fuzz boolean boundary loops, region fragments, and prepared/plain parity.
pub fn h_assert_boolean_boundary_pipeline(reader: &mut ByteReader<'_>) {
    let policy = h_policy();
    let fill_rule = HFillRule::NonZero;
    let a = h_region_from_bytes(reader, 2, 1);
    let b = h_region_from_bytes(reader, 2, 1);
    let op = h_boolean_op(reader);
    let prepared_a = a.prepare_topology_queries(&policy);
    let prepared_b = b.prepare_topology_queries(&policy);

    let plain_events = a.intersect_region(&b, &policy).unwrap();
    assert_eq!(
        prepared_a
            .intersect_prepared_region(&prepared_b, &policy)
            .unwrap(),
        plain_events
    );
    assert_eq!(
        prepared_a.intersect_region(&b.as_view(), &policy).unwrap(),
        plain_events
    );
    assert_eq!(
        a.as_view()
            .intersect_prepared_region(&prepared_b, &policy)
            .unwrap(),
        plain_events
    );

    if let HClassification::Decided(fragments) = plain_events
        .split_regions(&a.as_view(), &b.as_view(), &policy)
        .unwrap()
    {
        h_validate_region_fragments(&fragments, &policy);
        if let HClassification::Decided(selection) = fragments
            .classify_for_boolean(&a.as_view(), &b.as_view(), op, &policy)
            .unwrap()
        {
            assert!(
                selection.len()
                    <= fragments
                        .contours()
                        .iter()
                        .map(|fragments| { fragments.fragments.len() })
                        .sum()
            );
            let emitted = selection.emit_boundary_fragments(&fragments).unwrap();
            assert_eq!(
                emitted.directed_len() + emitted.unresolved_len(),
                selection
                    .classifications()
                    .iter()
                    .filter(|classification| classification.action.emits_fragment()
                        || matches!(
                            classification.action,
                            hypercurve::BooleanFragmentAction::BoundaryNeedsResolution
                        ))
                    .count()
            );
            if emitted.is_ready_for_traversal() {
                if let HClassification::Decided(chains) = emitted.assemble_chains(&policy) {
                    assert!(chains.closed_count() <= chains.len());
                    if let HClassification::Decided(loops) = chains.closed_loops() {
                        for contour in loops.to_contours(fill_rule).unwrap() {
                            h_assert_contour_finite(&contour);
                        }
                    }
                }
            }
        }
    }

    let plain_loops = a.boolean_boundary_loops(&b, op, &policy).unwrap();
    assert_eq!(
        prepared_a
            .boolean_boundary_loops(&prepared_b, op, &policy)
            .unwrap(),
        plain_loops
    );
    assert_eq!(
        prepared_a
            .boolean_boundary_loops_against_region(&b.as_view(), op, &policy)
            .unwrap(),
        plain_loops
    );
    assert_eq!(
        a.as_view()
            .boolean_boundary_loops_against_prepared_region(&prepared_b, op, &policy)
            .unwrap(),
        plain_loops
    );
    if let HClassification::Decided(loops) = &plain_loops {
        let report = match a
            .boolean_boundary_loop_report(&b, op, fill_rule, &policy)
            .unwrap()
        {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(reason) => {
                panic!("plain boundary loop report should match decided loops: {reason:?}")
            }
        };
        assert_eq!(
            prepared_a
                .boolean_boundary_loop_report(&prepared_b, op, fill_rule, &policy)
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(
            prepared_a
                .boolean_boundary_loop_report_against_region(&b.as_view(), op, fill_rule, &policy,)
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(
            a.as_view()
                .boolean_boundary_loop_report_against_prepared_region(
                    &prepared_b,
                    op,
                    fill_rule,
                    &policy,
                )
                .unwrap(),
            HClassification::Decided(report.clone())
        );
        assert_eq!(&report.loops, loops);
        assert_eq!(
            report.audit.is_valid(),
            matches!(
                report.audit.status,
                HBooleanBoundaryAuditStatus::Empty | HBooleanBoundaryAuditStatus::Valid
            )
        );
    }

    let plain_contours = a
        .boolean_boundary_contours(&b, op, fill_rule, &policy)
        .unwrap();
    assert_eq!(
        prepared_a
            .boolean_boundary_contours(&prepared_b, op, fill_rule, &policy)
            .unwrap(),
        plain_contours
    );
    assert_eq!(
        prepared_a
            .boolean_boundary_contours_against_region(&b.as_view(), op, fill_rule, &policy)
            .unwrap(),
        plain_contours
    );
    assert_eq!(
        a.as_view()
            .boolean_boundary_contours_against_prepared_region(&prepared_b, op, fill_rule, &policy,)
            .unwrap(),
        plain_contours
    );

    if let (HClassification::Decided(loops), HClassification::Decided(contours)) =
        (&plain_loops, &plain_contours)
    {
        h_assert_contour_boundary_sets_match(&loops.to_contours(fill_rule).unwrap(), contours);
    }

    let plain_region = a.boolean_region(&b, op, fill_rule, &policy).unwrap();
    assert_eq!(
        prepared_a
            .boolean_region(&prepared_b, op, fill_rule, &policy)
            .unwrap(),
        plain_region
    );
    assert_eq!(
        prepared_a
            .boolean_region_against_region(&b.as_view(), op, fill_rule, &policy)
            .unwrap(),
        plain_region
    );
    assert_eq!(
        a.as_view()
            .boolean_region_against_prepared_region(&prepared_b, op, fill_rule, &policy)
            .unwrap(),
        plain_region
    );

    if let HClassification::Decided(result) = &plain_region {
        h_assert_region_semantics(&a, &b, result, op);
    }
    if let (HClassification::Decided(contours), HClassification::Decided(region)) =
        (&plain_contours, &plain_region)
    {
        if let HClassification::Decided(rebuilt) =
            HRegion2::from_boundary_contours(contours.clone(), &policy).unwrap()
        {
            assert_eq!(rebuilt, *region);
            let report =
                match HRegion2::from_boundary_contours_report(contours.clone(), &policy).unwrap() {
                    HClassification::Decided(report) => report,
                    HClassification::Uncertain(reason) => {
                        panic!("decided contour nesting should also have a report: {reason:?}")
                    }
                };
            assert_eq!(&report.result, region);
            assert_eq!(
                report.audit.status,
                if contours.is_empty() {
                    HBoundaryContourNestingStatus::Empty
                } else {
                    HBoundaryContourNestingStatus::Valid
                }
            );
            assert!(report.audit.is_valid());
            assert_eq!(report.audit.input_contour_count, contours.len());
            assert_eq!(
                report.audit.material_contour_count + report.audit.hole_contour_count,
                contours.len()
            );
        }
    }
}

fn h_l_path_curve(reader: &mut ByteReader<'_>) -> (HCurveString2, HRealValue) {
    let horizontal = reader.i32_range(6, 96);
    let vertical = reader.i32_range(6, 96);
    let distance = reader.i32_range(1, (horizontal.min(vertical) / 3).max(1));
    let curve = HCurveString2::try_new(vec![
        HSegment2::Line(h_line_from_i32((0, 0), (horizontal, 0))),
        HSegment2::Line(h_line_from_i32((horizontal, 0), (horizontal, vertical))),
    ])
    .unwrap();
    (curve, h_scalar_i32(distance))
}

fn h_polygon_reconstruction_options() -> HPolylineReconstructionOptions {
    let mut options = HPolylineReconstructionOptions::default();
    // These inputs are intentionally polygonal. Disabling arc promotion keeps
    // this harness focused on closed-polyline reconstruction, clipping, and
    // raw offset topology instead of sampled circle fitting.
    options.min_arc_points = 64;
    options.distance_tolerance = 1e-8;
    options.duplicate_point_tolerance = 1e-12;
    options
}

fn h_contour_from_i32_points(points: &[(i32, i32)]) -> HContour {
    let vertices: Vec<_> = points.iter().map(|&(x, y)| h_vertex_i32(x, y, 0)).collect();
    HContour2::from_bulge_vertices_with_fill_rule(&vertices, HFillRule::NonZero).unwrap()
}

fn h_rectangle_i32(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> HContour {
    h_contour_from_i32_points(&[(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)])
}

fn h_large_concavity(width: i32, height: i32, throat: i32) -> Vec<(i32, i32)> {
    let arm = throat.max(1);
    vec![
        (0, 0),
        (width, 0),
        (width, height),
        (width - arm, height),
        (width - arm, arm),
        (arm, arm),
        (arm, height),
        (0, height),
    ]
}

fn h_slender_concavity(width: i32, height: i32, slot_x: i32) -> Vec<(i32, i32)> {
    let left = slot_x.clamp(1, width - 2);
    let right = left + 1;
    vec![
        (0, 0),
        (width, 0),
        (width, height),
        (right, height),
        (right, 1),
        (left, 1),
        (left, height),
        (0, height),
    ]
}

fn h_comb_concavity(width: i32, height: i32, teeth: usize) -> Vec<(i32, i32)> {
    let mut points = vec![(0, 0), (width, 0), (width, height)];
    let step = (width / (teeth as i32 * 2 + 1)).max(1);
    for tooth in (0..teeth).rev() {
        let x_outer = (2 * tooth as i32 + 2) * step;
        let x_inner = (2 * tooth as i32 + 1) * step;
        points.push((x_outer, height));
        points.push((x_outer, 1));
        points.push((x_inner, 1));
        points.push((x_inner, height));
    }
    points.push((0, height));
    points
}

fn h_bowtie(size: i32) -> Vec<(i32, i32)> {
    vec![(0, 0), (size, size), (0, size), (size, 0)]
}

fn h_adversarial_polygon_points(reader: &mut ByteReader<'_>) -> Vec<(i32, i32)> {
    let width = reader.i32_range(12, 96);
    let height = reader.i32_range(12, 96);
    let offset = reader.i32_range(1, 32);
    match reader.byte() % 5 {
        0 => h_large_concavity(width, height, offset.min(width.min(height) / 3).max(2)),
        1 => h_slender_concavity(width, height, offset.min(width - 2)),
        2 => h_comb_concavity(width, height, reader.usize_range(2, 5)),
        3 => {
            let mut points = h_large_concavity(width, height, 2);
            points.splice(3..3, [(width / 2, height + offset)]);
            points
        }
        _ => h_bowtie(width.min(height)),
    }
}

fn h_exercise_adversarial_offset(contour: &HContour, distance: HRealValue, policy: &HCurvePolicy) {
    h_assert_contour_finite(contour);
    let _ = contour.has_self_contacts(policy).unwrap();
    if let HClassification::Decided(raw) = contour
        .offset_left_with_line_joins(distance.clone(), policy)
        .unwrap()
    {
        h_assert_contour_finite(&raw);
    }
    if let HClassification::Decided(checked) =
        contour.offset_left_checked(distance, policy).unwrap()
    {
        h_assert_contour_finite(&checked);
        assert_eq!(
            checked.has_self_contacts(policy).unwrap(),
            HClassification::Decided(false)
        );
    }
}

/// Fuzz adversarial polygon offsetting, clipping, and reconstruction.
pub fn h_assert_adversarial_polygon_pipeline(reader: &mut ByteReader<'_>) {
    let policy = h_policy();
    let points = h_adversarial_polygon_points(reader);
    let material = h_contour_from_i32_points(&points);
    let width = points
        .iter()
        .map(|point| point.0)
        .max()
        .unwrap_or(16)
        .max(16);
    let height = points
        .iter()
        .map(|point| point.1)
        .max()
        .unwrap_or(16)
        .max(16);
    let hole = h_rectangle_i32(
        width / 3,
        2,
        (width / 3 + 2).min(width - 2),
        4.min(height - 2),
    );
    let region = HRegion2::new(vec![material.clone()], vec![hole.clone()]);
    let cutter = HRegion2::from_material_contours(vec![h_rectangle_i32(
        width / 4,
        -1,
        width + reader.i32_range(2, 32),
        (height / 2).max(3),
    )]);

    let distance = h_scalar_i32(reader.i32_range(1, 3));
    h_exercise_adversarial_offset(&material, distance.clone(), &policy);
    h_exercise_adversarial_offset(&hole, -distance.clone(), &policy);

    let prepared_region = region.prepare_topology_queries(&policy);
    let prepared_cutter = cutter.prepare_topology_queries(&policy);
    for op in [
        HBooleanOp::Union,
        HBooleanOp::Intersection,
        HBooleanOp::Difference,
        HBooleanOp::Xor,
    ] {
        let contours = region
            .boolean_boundary_contours(&cutter, op, HFillRule::NonZero, &policy)
            .unwrap();
        assert_eq!(
            prepared_region
                .boolean_boundary_contours(&prepared_cutter, op, HFillRule::NonZero, &policy)
                .unwrap(),
            contours
        );
        if let HClassification::Decided(contours) = &contours {
            for contour in contours {
                h_assert_contour_finite(contour);
            }
        }

        let boolean_region = region
            .boolean_region(&cutter, op, HFillRule::NonZero, &policy)
            .unwrap();
        assert_eq!(
            prepared_region
                .boolean_region(&prepared_cutter, op, HFillRule::NonZero, &policy)
                .unwrap(),
            boolean_region
        );
        if let HClassification::Decided(result) = &boolean_region {
            h_assert_region_finite(result);
        }
    }

    let mut samples: Vec<_> = points.iter().map(|&(x, y)| h_point_i32(x, y)).collect();
    if samples.len() > 3 {
        samples.insert(1, samples[1].clone());
        samples.push(samples[0].clone());
    }
    let reconstructed =
        HContour2::reconstruct_from_closed_polyline(&samples, h_polygon_reconstruction_options())
            .unwrap();
    h_assert_contour_finite(&reconstructed);
    let _ = reconstructed.intersect_self(&policy).unwrap();
    h_exercise_adversarial_offset(&reconstructed, h_scalar_i32(1), &policy);
}

/// Fuzz every public offset-outline cap path on line and arc curve strings.
pub fn h_assert_offset_cap_matrix(reader: &mut ByteReader<'_>) {
    let policy = h_policy();
    let (curve, distance) = h_l_path_curve(reader);

    for cap in [HOffsetCap::Round, HOffsetCap::Butt, HOffsetCap::Square] {
        let dispatched = curve
            .offset_outline(distance.clone(), cap, &policy)
            .unwrap();
        let direct = match cap {
            HOffsetCap::Round => curve.offset_outline_round_caps(distance.clone(), &policy),
            HOffsetCap::Butt => curve.offset_outline_butt_caps(distance.clone(), &policy),
            HOffsetCap::Square => curve.offset_outline_square_caps(distance.clone(), &policy),
        }
        .unwrap();
        assert_eq!(dispatched, direct);
        let HClassification::Decided(outline) = dispatched else {
            panic!("simple L-path outline should decide for cap {cap:?}");
        };
        h_assert_contour_finite(&outline);
        assert_eq!(
            outline.has_self_contacts(&policy).unwrap(),
            HClassification::Decided(false)
        );
    }

    let radius = reader.i32_range(4, 64);
    let distance = h_scalar_i32(reader.i32_range(1, radius - 1));
    let arc_curve = HCurveString2::try_new(vec![
        HSegment2::from_bulge(
            h_point_i32(-radius, 0),
            h_point_i32(radius, 0),
            h_scalar_i32(-1),
        )
        .unwrap(),
    ])
    .unwrap();

    for cap in [HOffsetCap::Round, HOffsetCap::Butt, HOffsetCap::Square] {
        if let HClassification::Decided(outline) = arc_curve
            .offset_outline(distance.clone(), cap, &policy)
            .unwrap()
        {
            h_assert_contour_finite(&outline);
        }
    }
}

/// Fuzz sampled-polyline reconstruction into open curve strings and closed contours.
pub fn h_assert_polyline_reconstruction(reader: &mut ByteReader<'_>) {
    let mut options = HPolylineReconstructionOptions::default();
    options.min_arc_points = reader.usize_range(3, 5);
    options.distance_tolerance = 1e-6;

    let points = match reader.byte() % 3 {
        0 => {
            let count = reader.usize_range(2, 24);
            let dx = reader.f64_range(0.25, 4.0);
            let dy = reader.f64_range(-0.25, 0.25);
            (0..count)
                .map(|index| h_point(index as f64 * dx, index as f64 * dy))
                .collect::<Vec<_>>()
        }
        1 => {
            let count = reader.usize_range(3, 24);
            let cx = reader.f64_range(-8.0, 8.0);
            let cy = reader.f64_range(-8.0, 8.0);
            let radius = reader.f64_range(0.5, 16.0);
            let start = reader.f64_range(-PI, PI);
            let sweep = reader.f64_range(PI / 12.0, PI);
            let direction = if reader.bool() { 1.0 } else { -1.0 };
            (0..count)
                .map(|index| {
                    let t = start + direction * sweep * index as f64 / (count - 1) as f64;
                    h_point(cx + radius * t.cos(), cy + radius * t.sin())
                })
                .collect::<Vec<_>>()
        }
        _ => {
            let count = reader.usize_range(3, 24);
            (0..count)
                .map(|index| {
                    let x = index as f64;
                    let y = if index % 2 == 0 {
                        0.0
                    } else {
                        reader.f64_range(0.25, 2.0)
                    };
                    h_point(x, y)
                })
                .collect::<Vec<_>>()
        }
    };

    let curve = HCurveString2::reconstruct_from_polyline(&points, options).unwrap();
    h_assert_curve_string_finite(&curve);
    assert!(curve.len() <= points.len().saturating_sub(1));

    let rectangle = [
        h_point(0.0, 0.0),
        h_point(reader.f64_range(1.0, 16.0), 0.0),
        h_point(reader.f64_range(1.0, 16.0), reader.f64_range(1.0, 16.0)),
        h_point(0.0, reader.f64_range(1.0, 16.0)),
    ];
    let contour = HContour2::reconstruct_from_closed_polyline(&rectangle, options).unwrap();
    h_assert_contour_finite(&contour);
}

fn h_quadratic_through_point_at(
    parameter: HRealValue,
    target: &HPoint,
    first_offset: (i32, i32),
    second_offset: (i32, i32),
) -> HQuadraticBezier2 {
    let one_minus_t = HReal::one() - &parameter;
    let b0 = &one_minus_t * &one_minus_t;
    let b1 = HReal::from(2_i8) * &parameter * &one_minus_t;
    let b2 = &parameter * &parameter;
    let p0 = HPoint2::new(
        target.x() + &HReal::from(first_offset.0),
        target.y() + &HReal::from(first_offset.1),
    );
    let p1 = HPoint2::new(
        target.x() + &HReal::from(second_offset.0),
        target.y() + &HReal::from(second_offset.1),
    );
    let numerator_x = target.x() - &(&b0 * p0.x()) - &(&b1 * p1.x());
    let numerator_y = target.y() - &(&b0 * p0.y()) - &(&b1 * p1.y());
    let p2_x = (numerator_x / &b2)
        .expect("nonzero parameter gives nonzero quadratic Bernstein endpoint weight");
    let p2_y = (numerator_y / &b2)
        .expect("nonzero parameter gives nonzero quadratic Bernstein endpoint weight");
    HQuadraticBezier2::new(p0, p1, HPoint2::new(p2_x, p2_y))
}

fn h_cubic_through_point_at(
    parameter: HRealValue,
    target: &HPoint,
    first_offset: (i32, i32),
    second_offset: (i32, i32),
    third_offset: (i32, i32),
) -> HCubicBezier2 {
    let one_minus_t = HReal::one() - &parameter;
    let b0 = &one_minus_t * &one_minus_t * &one_minus_t;
    let b1 = HReal::from(3_i8) * &parameter * &one_minus_t * &one_minus_t;
    let b2 = HReal::from(3_i8) * &parameter * &parameter * &one_minus_t;
    let b3 = &parameter * &parameter * &parameter;
    let p0 = HPoint2::new(
        target.x() + &HReal::from(first_offset.0),
        target.y() + &HReal::from(first_offset.1),
    );
    let p1 = HPoint2::new(
        target.x() + &HReal::from(second_offset.0),
        target.y() + &HReal::from(second_offset.1),
    );
    let p2 = HPoint2::new(
        target.x() + &HReal::from(third_offset.0),
        target.y() + &HReal::from(third_offset.1),
    );
    let numerator_x = target.x() - &(&b0 * p0.x()) - &(&b1 * p1.x()) - &(&b2 * p2.x());
    let numerator_y = target.y() - &(&b0 * p0.y()) - &(&b1 * p1.y()) - &(&b2 * p2.y());
    let p3_x = (numerator_x / &b3)
        .expect("nonzero parameter gives nonzero cubic Bernstein endpoint weight");
    let p3_y = (numerator_y / &b3)
        .expect("nonzero parameter gives nonzero cubic Bernstein endpoint weight");
    HCubicBezier2::new(p0, p1, p2, HPoint2::new(p3_x, p3_y))
}

fn h_assert_bezier_conic_relations(reader: &mut ByteReader<'_>) {
    let policy = h_policy();
    let lift = reader.i32_range(1, 16);
    let parameter = h_ratio(reader.i32_range(1, 511), 512);
    let rational = HRationalQuadraticBezier2::try_new(
        h_point_i32(0, 0),
        h_point_i32(256, lift),
        h_point_i32(512, 0),
        HReal::one(),
        HReal::from(2_i8),
        HReal::one(),
    )
    .unwrap();
    let target = match rational.point_at(parameter.clone(), &policy) {
        HClassification::Decided(point) => point,
        HClassification::Uncertain(_) => return,
    };
    let polynomial = h_quadratic_through_point_at(parameter.clone(), &target, (5, 7), (-3, 11));
    let cubic = h_cubic_through_point_at(parameter, &target, (5, 7), (-3, 11), (13, -5));

    match rational.relation_to_quadratic(&polynomial, &policy) {
        HClassification::Decided(HBezierCurveRelation::IntersectionPoints { points }) => {
            assert!(points.iter().any(|point| point.point() == &target));
            for point in points {
                h_assert_point_finite(point.point());
            }
        }
        relation => {
            panic!("dyadic rational/polynomial same-parameter hit was not promoted: {relation:?}");
        }
    }
    match rational.relation_to_cubic(&cubic, &policy) {
        HClassification::Decided(HBezierCurveRelation::IntersectionPoints { points }) => {
            assert!(points.iter().any(|point| point.point() == &target));
            for point in points {
                h_assert_point_finite(point.point());
            }
        }
        relation => {
            panic!("dyadic rational/cubic same-parameter hit was not promoted: {relation:?}");
        }
    }

    let shared_endpoint_first = HQuadraticBezier2::new(
        h_point_i32(0, 0),
        h_point_i32(1, 2 * lift),
        h_point_i32(2, 0),
    );
    let shared_endpoint_second = HQuadraticBezier2::new(
        h_point_i32(0, 0),
        h_point_i32(1, 0),
        h_point_i32(2, 4 * lift),
    );
    let midpoint = HPoint2::new(HReal::one(), HReal::from(lift));
    match shared_endpoint_first.relation_to_quadratic(&shared_endpoint_second, &policy) {
        HClassification::Decided(HBezierCurveRelation::IntersectionPoints { points }) => {
            assert!(
                points
                    .iter()
                    .any(|point| point.point() == &h_point_i32(0, 0))
            );
            assert!(points.iter().any(|point| point.point() == &midpoint));
            for point in points {
                h_assert_point_finite(point.point());
            }
        }
        relation => {
            panic!("shared endpoint hid generated quadratic midpoint crossing: {relation:?}");
        }
    }
    let rational_shared_endpoint_first = HRationalQuadraticBezier2::try_new(
        h_point_i32(0, 0),
        h_point_i32(1, 2 * lift),
        h_point_i32(2, 0),
        HReal::one(),
        HReal::from(2_i8),
        HReal::one(),
    )
    .unwrap();
    let rational_shared_endpoint_second = HRationalQuadraticBezier2::try_new(
        h_point_i32(0, 0),
        h_point_i32(1, 0),
        h_point_i32(2, 8 * lift),
        HReal::one(),
        HReal::from(2_i8),
        HReal::one(),
    )
    .unwrap();
    let rational_midpoint = HPoint2::new(HReal::one(), h_ratio(4 * lift, 3));
    match rational_shared_endpoint_first
        .relation_to_rational_quadratic(&rational_shared_endpoint_second, &policy)
    {
        HClassification::Decided(HBezierCurveRelation::IntersectionPoints { points }) => {
            assert!(
                points
                    .iter()
                    .any(|point| point.point() == &h_point_i32(0, 0))
            );
            assert!(
                points
                    .iter()
                    .any(|point| point.point() == &rational_midpoint)
            );
            for point in points {
                h_assert_point_finite(point.point());
            }
        }
        relation => {
            panic!("shared endpoint hid generated rational midpoint crossing: {relation:?}");
        }
    }

    let graph_gap = HRationalQuadraticBezier2::try_new(
        h_point_i32(0, 1),
        h_point_i32(256, lift + 1),
        h_point_i32(512, 1),
        HReal::one(),
        HReal::from(2_i8),
        HReal::one(),
    )
    .unwrap();
    match rational.relation_to_rational_quadratic(&graph_gap, &policy) {
        HClassification::Decided(HBezierCurveRelation::NoIntersection) => {}
        relation => {
            panic!("matching-weight rational graph gap was not certified as no-hit: {relation:?}");
        }
    }
    match rational.graph_order_to_rational_quadratic_over_axis(&graph_gap, HAxis2::X, &policy) {
        HClassification::Decided(HBezierMonotoneGraphOrder::FirstLess) => {}
        relation => {
            panic!(
                "matching-weight rational graph order did not certify the generated gap: {relation:?}"
            );
        }
    }
    let equal_weight_rational = HRationalQuadraticBezier2::try_new(
        h_point_i32(0, 0),
        h_point_i32(256, lift),
        h_point_i32(512, 0),
        HReal::from(-3_i8),
        HReal::from(-3_i8),
        HReal::from(-3_i8),
    )
    .unwrap();
    let polynomial_graph_gap = HQuadraticBezier2::new(
        h_point_i32(0, 1),
        h_point_i32(256, lift + 1),
        h_point_i32(512, 1),
    );
    match equal_weight_rational.graph_order_to_quadratic_over_axis(
        &polynomial_graph_gap,
        HAxis2::X,
        &policy,
    ) {
        HClassification::Decided(HBezierMonotoneGraphOrder::FirstLess) => {}
        relation => {
            panic!(
                "equal-weight rational/polynomial graph order did not certify the generated gap: {relation:?}"
            );
        }
    }
    match polynomial_graph_gap.graph_order_to_rational_quadratic_over_axis(
        &equal_weight_rational,
        HAxis2::X,
        &policy,
    ) {
        HClassification::Decided(HBezierMonotoneGraphOrder::FirstGreater) => {}
        relation => {
            panic!(
                "polynomial/equal-weight rational graph order did not certify the generated gap: {relation:?}"
            );
        }
    }
    let non_equal_rational_graph = HRationalQuadraticBezier2::try_new(
        HPoint2::new(HReal::zero(), HReal::one()),
        HPoint2::new(h_ratio(1, 4), HReal::one()),
        HPoint2::new(HReal::one(), HReal::one()),
        HReal::one(),
        HReal::from(2_i8),
        HReal::from(3_i8),
    )
    .unwrap();
    let non_equal_polynomial_baseline = HQuadraticBezier2::new(
        HPoint2::new(HReal::zero(), HReal::zero()),
        HPoint2::new(h_ratio(1, 2), HReal::zero()),
        HPoint2::new(HReal::one(), HReal::zero()),
    );
    match non_equal_rational_graph.graph_order_to_quadratic_over_axis(
        &non_equal_polynomial_baseline,
        HAxis2::X,
        &policy,
    ) {
        HClassification::Decided(HBezierMonotoneGraphOrder::FirstGreater) => {}
        relation => {
            panic!(
                "non-equal rational/polynomial strict graph order did not certify the generated gap: {relation:?}"
            );
        }
    }
    match non_equal_polynomial_baseline.graph_order_to_rational_quadratic_over_axis(
        &non_equal_rational_graph,
        HAxis2::X,
        &policy,
    ) {
        HClassification::Decided(HBezierMonotoneGraphOrder::FirstLess) => {}
        relation => {
            panic!(
                "polynomial/non-equal rational strict graph order did not certify the generated gap: {relation:?}"
            );
        }
    }
    match non_equal_rational_graph.relation_to_quadratic(&non_equal_polynomial_baseline, &policy) {
        HClassification::Decided(HBezierCurveRelation::NoIntersection) => {}
        relation => {
            panic!(
                "non-equal rational/polynomial strict graph relation did not certify no-hit: {relation:?}"
            );
        }
    }
    let non_equal_polynomial_crossing = HQuadraticBezier2::new(
        HPoint2::new(HReal::zero(), HReal::from(2_i8)),
        HPoint2::new(h_ratio(1, 2), HReal::zero()),
        HPoint2::new(HReal::one(), HReal::from(2_i8)),
    );
    match non_equal_rational_graph.graph_order_to_quadratic_over_axis(
        &non_equal_polynomial_crossing,
        HAxis2::X,
        &policy,
    ) {
        HClassification::Decided(HBezierMonotoneGraphOrder::IntersectsOrTouches {
            parameters,
            spans,
        }) if !parameters.is_empty() || !spans.is_empty() => {}
        relation => {
            panic!(
                "non-equal rational/polynomial quartic graph roots were not retained: {relation:?}"
            );
        }
    }
    match non_equal_rational_graph.relation_to_quadratic(&non_equal_polynomial_crossing, &policy) {
        HClassification::Decided(HBezierCurveRelation::IntersectionPoints { .. })
        | HClassification::Decided(HBezierCurveRelation::IntersectionRegions { .. }) => {}
        relation => {
            panic!(
                "non-equal rational/polynomial quartic graph roots did not reach relation dispatch: {relation:?}"
            );
        }
    }
    let non_equal_cubic_baseline = HCubicBezier2::new(
        HPoint2::new(HReal::zero(), HReal::zero()),
        HPoint2::new(h_ratio(1, 3), HReal::zero()),
        HPoint2::new(h_ratio(2, 3), HReal::zero()),
        HPoint2::new(HReal::one(), HReal::zero()),
    );
    match non_equal_rational_graph.graph_order_to_cubic_over_axis(
        &non_equal_cubic_baseline,
        HAxis2::X,
        &policy,
    ) {
        HClassification::Decided(HBezierMonotoneGraphOrder::FirstGreater) => {}
        relation => {
            panic!(
                "non-equal rational/cubic strict graph order did not certify the generated gap: {relation:?}"
            );
        }
    }
    match non_equal_cubic_baseline.graph_order_to_rational_quadratic_over_axis(
        &non_equal_rational_graph,
        HAxis2::X,
        &policy,
    ) {
        HClassification::Decided(HBezierMonotoneGraphOrder::FirstLess) => {}
        relation => {
            panic!(
                "cubic/non-equal rational strict graph order did not certify the generated gap: {relation:?}"
            );
        }
    }
    match non_equal_rational_graph.relation_to_cubic(&non_equal_cubic_baseline, &policy) {
        HClassification::Decided(HBezierCurveRelation::NoIntersection) => {}
        relation => {
            panic!(
                "non-equal rational/cubic strict graph relation did not certify no-hit: {relation:?}"
            );
        }
    }
    let non_equal_cubic_crossing = HCubicBezier2::new(
        HPoint2::new(HReal::zero(), HReal::from(2_i8)),
        HPoint2::new(h_ratio(1, 3), HReal::zero()),
        HPoint2::new(h_ratio(2, 3), HReal::zero()),
        HPoint2::new(HReal::one(), HReal::from(2_i8)),
    );
    match non_equal_rational_graph.graph_order_to_cubic_over_axis(
        &non_equal_cubic_crossing,
        HAxis2::X,
        &policy,
    ) {
        HClassification::Decided(HBezierMonotoneGraphOrder::IntersectsOrTouches {
            parameters,
            spans,
        }) if !parameters.is_empty() || !spans.is_empty() => {}
        relation => {
            panic!("non-equal rational/cubic quintic graph roots were not retained: {relation:?}");
        }
    }
    match non_equal_rational_graph.relation_to_cubic(&non_equal_cubic_crossing, &policy) {
        HClassification::Decided(HBezierCurveRelation::IntersectionPoints { .. })
        | HClassification::Decided(HBezierCurveRelation::IntersectionRegions { .. }) => {}
        relation => {
            panic!(
                "non-equal rational/cubic quintic graph roots did not reach relation dispatch: {relation:?}"
            );
        }
    }
    let equal_weight_rational_cubic = HRationalQuadraticBezier2::try_new(
        h_point_i32(0, 0),
        h_point_i32(300, 3 * lift),
        h_point_i32(600, 0),
        HReal::from(-3_i8),
        HReal::from(-3_i8),
        HReal::from(-3_i8),
    )
    .unwrap();
    let cubic_graph_gap = HCubicBezier2::new(
        h_point_i32(0, 1),
        h_point_i32(200, 2 * lift + 1),
        h_point_i32(400, 2 * lift + 1),
        h_point_i32(600, 1),
    );
    match equal_weight_rational_cubic.graph_order_to_cubic_over_axis(
        &cubic_graph_gap,
        HAxis2::X,
        &policy,
    ) {
        HClassification::Decided(HBezierMonotoneGraphOrder::FirstLess) => {}
        relation => {
            panic!(
                "equal-weight rational/cubic graph order did not certify the generated gap: {relation:?}"
            );
        }
    }
    match cubic_graph_gap.graph_order_to_rational_quadratic_over_axis(
        &equal_weight_rational_cubic,
        HAxis2::X,
        &policy,
    ) {
        HClassification::Decided(HBezierMonotoneGraphOrder::FirstGreater) => {}
        relation => {
            panic!(
                "cubic/equal-weight rational graph order did not certify the generated gap: {relation:?}"
            );
        }
    }

    let reversed = HRationalQuadraticBezier2::try_new(
        h_point_i32(512, 0),
        h_point_i32(256, lift),
        h_point_i32(0, 0),
        HReal::one(),
        HReal::from(2_i8),
        HReal::one(),
    )
    .unwrap();
    match rational.relation_to_rational_quadratic(&reversed, &policy) {
        HClassification::Decided(HBezierCurveRelation::SameCurveImage) => {}
        relation => {
            panic!("reversed rational conic image was not certified exactly: {relation:?}");
        }
    }

    let negative_line = HRationalQuadraticBezier2::try_new(
        h_point_i32(0, 0),
        h_point_i32(256, 0),
        h_point_i32(512, 0),
        HReal::from(-1_i8),
        HReal::from(-2_i8),
        HReal::from(-1_i8),
    )
    .unwrap();
    let negative_cross_line = HRationalQuadraticBezier2::try_new(
        h_point_i32(256, -16),
        h_point_i32(256, 0),
        h_point_i32(256, 16),
        HReal::from(-1_i8),
        HReal::from(-2_i8),
        HReal::from(-1_i8),
    )
    .unwrap();
    match negative_line.relation_to_rational_quadratic(&negative_cross_line, &policy) {
        HClassification::Decided(HBezierCurveRelation::LineSegmentIntersection { .. }) => {}
        relation => {
            panic!(
                "same-sign negative rational line images did not dispatch exactly: {relation:?}"
            );
        }
    }

    let polynomial_line =
        HQuadraticBezier2::new(h_point_i32(-1, 0), h_point_i32(128, 0), h_point_i32(512, 0));
    let isolated_cubic = HCubicBezier2::new(
        h_point_i32(0, -1),
        h_point_i32(170, 1),
        h_point_i32(340, 1),
        h_point_i32(512, 1),
    );
    match polynomial_line.relation_to_cubic(&isolated_cubic, &policy) {
        HClassification::Decided(HBezierCurveRelation::IntersectionRegions { regions })
            if !regions.is_empty() => {}
        relation => {
            panic!("line-image cubic isolated roots were not retained as regions: {relation:?}");
        }
    }

    let graph_lower = HQuadraticBezier2::new(
        h_point_i32(0, 0),
        h_point_i32(3, 3 * lift),
        h_point_i32(6, 0),
    );
    let graph_upper = HCubicBezier2::new(
        h_point_i32(0, 1),
        h_point_i32(2, 2 * lift + 1),
        h_point_i32(4, 2 * lift + 1),
        h_point_i32(6, 1),
    );
    match graph_lower.graph_order_to_cubic_over_axis(&graph_upper, HAxis2::X, &policy) {
        HClassification::Decided(HBezierMonotoneGraphOrder::FirstLess) => {}
        relation => {
            panic!("shared-axis graph order did not certify the generated gap: {relation:?}")
        }
    }

    let tangent_y = (HReal::from(3 * lift) / HReal::from(2_i8)).unwrap();
    let tangent_line = HLineSeg2::try_new(
        HPoint2::new(HReal::zero(), tangent_y.clone()),
        HPoint2::new(HReal::from(6_i8), tangent_y),
    )
    .unwrap();
    match graph_lower.relation_to_line_with_contacts(&tangent_line, &policy) {
        HClassification::Decided(HBezierLineContactRelation::Contacts { contacts })
            if contacts.iter().any(|contact| {
                contact.kind() == HBezierLineContactKind::Tangent
                    && contact.parameter() == &h_ratio(1, 2)
            }) => {}
        relation => panic!("generated quadratic tangent was not classified exactly: {relation:?}"),
    }

    let rational_tangent_y = (HReal::from(2 * lift) / HReal::from(3_i8)).unwrap();
    let rational_tangent = HLineSeg2::try_new(
        HPoint2::new(HReal::zero(), rational_tangent_y.clone()),
        HPoint2::new(HReal::from(512_i32), rational_tangent_y),
    )
    .unwrap();
    match rational.relation_to_line_with_contacts(&rational_tangent, &policy) {
        HClassification::Decided(HBezierLineContactRelation::Contacts { contacts })
            if contacts.iter().any(|contact| {
                contact.kind() == HBezierLineContactKind::Tangent
                    && contact.parameter() == &h_ratio(1, 2)
            }) => {}
        relation => {
            panic!("generated rational conic tangent was not classified exactly: {relation:?}")
        }
    }
}

fn h_assert_bezier_point_image_fits(reader: &mut ByteReader<'_>) {
    let policy = h_policy();
    let point = h_point_i32(reader.i32_range(-16, 16), reader.i32_range(-16, 16));
    let sign = if reader.bool() { 1 } else { -1 };
    let endpoint_weight = HReal::from(sign);
    let control_weight = HReal::from(reader.i32_range(1, 8) * sign);
    let quadratic = HQuadraticBezier2::new(point.clone(), point.clone(), point.clone());
    let cubic = HCubicBezier2::new(point.clone(), point.clone(), point.clone(), point.clone());
    let rational = HRationalQuadraticBezier2::try_new(
        point.clone(),
        point.clone(),
        point.clone(),
        endpoint_weight.clone(),
        control_weight,
        endpoint_weight,
    )
    .unwrap();

    for (count, fit) in [
        (3, quadratic.fit_exact_point_image(&policy).unwrap()),
        (4, cubic.fit_exact_point_image(&policy).unwrap()),
        (3, rational.fit_exact_point_image(&policy).unwrap()),
    ] {
        match fit {
            HClassification::Decided(HBezierPointImageFitRelation::Fit(fit)) => {
                assert_eq!(fit.point(), &point);
                assert_eq!(fit.control_point_count(), count);
                assert_eq!(fit.fit_certificate().source_start(), 0);
                assert_eq!(fit.fit_certificate().source_end(), count);
                assert_eq!(fit.fit_certificate().fit_error_bound(), &HReal::zero());
                assert_eq!(fit.fit_certificate().source_flattening_error(), None);
                assert_eq!(
                    fit.fit_certificate().metric(),
                    HBezierFitErrorMetric::ExactEuclideanDistance
                );
                assert_eq!(
                    fit.fit_certificate().bound_kind(),
                    HBezierFitBoundKind::ProvenExact
                );
            }
            relation => panic!("collapsed Bezier/conic did not certify point image: {relation:?}"),
        }
    }
    let HClassification::Decided(preflight) = cubic.offset_preflight(&policy) else {
        panic!("collapsed cubic offset preflight should be decided");
    };
    assert!(
        preflight
            .risks()
            .contains(&HBezierOffsetRisk::DegeneratePoint)
    );
    assert_eq!(preflight.construction_policy(), &policy);

    match quadratic.relation_to_rational_quadratic(&rational, &policy) {
        HClassification::Decided(HBezierCurveRelation::SameControlPolygon)
        | HClassification::Decided(HBezierCurveRelation::SameCurveImage)
        | HClassification::Decided(HBezierCurveRelation::IntersectionPoints { .. }) => {}
        relation => panic!("collapsed polynomial/rational point images diverged: {relation:?}"),
    }

    let options = hypercurve::BezierFlatteningOptions::try_new(HReal::one(), 4, &policy).unwrap();
    let HClassification::Decided(polyline) = cubic.flatten_certified(&options, &policy) else {
        panic!("collapsed cubic should flatten under generous fuzz options");
    };
    match polyline.fit_exact_point(&policy).unwrap() {
        HClassification::Decided(HBezierPointFitRelation::Fit(fit)) => {
            assert_eq!(fit.point(), &point);
            assert_eq!(fit.source_certificate(), polyline.certificate());
            assert_eq!(fit.fit_certificate().source_end(), polyline.points().len());
            assert_eq!(fit.fit_certificate().fit_error_bound(), &HReal::zero());
            assert_eq!(
                fit.fit_certificate().source_flattening_error(),
                Some(polyline.certificate().max_error())
            );
        }
        relation => panic!("collapsed certified polyline did not fit one point: {relation:?}"),
    }

    let line_curve =
        HQuadraticBezier2::new(h_point_i32(-2, 0), h_point_i32(0, 0), h_point_i32(2, 0));
    let HClassification::Decided(preflight) = line_curve.offset_preflight(&policy) else {
        panic!("line-like quadratic offset preflight should be decided");
    };
    assert!(preflight.is_clear());
    match line_curve.fit_exact_line_image(&policy).unwrap() {
        HClassification::Decided(HBezierLineImageFitRelation::Fit(fit)) => {
            let right_offset = fit.offset_right_exact(HReal::one()).unwrap();
            assert_eq!(right_offset.control_point_count(), 3);
            assert_eq!(right_offset.distance(), &HReal::from(-1_i8));
            assert_eq!(right_offset.fit_certificate(), fit.fit_certificate());
        }
        relation => panic!("line-like quadratic image fit was not exact: {relation:?}"),
    }
    match line_curve
        .offset_left_staged(HReal::one(), &policy)
        .unwrap()
    {
        HClassification::Decided(candidate) => {
            let HBezierOffsetCandidate2::ExactLineImage { preflight, .. } = &candidate else {
                panic!("line-like quadratic staged offset was not exact: {candidate:?}");
            };
            assert!(preflight.is_clear());
            assert_eq!(candidate.preflight(), preflight);
            let Some(offset) = candidate.exact_line_image_offset() else {
                unreachable!("matched exact line image");
            };
            assert_eq!(offset.control_point_count(), 3);
            assert_eq!(offset.distance(), &HReal::one());
            assert_eq!(candidate.distance(), &HReal::one());
        }
        relation => panic!("line-like quadratic staged offset was not exact: {relation:?}"),
    }
    match line_curve
        .offset_right_staged(HReal::one(), &policy)
        .unwrap()
    {
        HClassification::Decided(candidate) => {
            let HBezierOffsetCandidate2::ExactLineImage { preflight, .. } = &candidate else {
                panic!("line-like quadratic staged right offset was not exact: {candidate:?}");
            };
            assert!(preflight.is_clear());
            assert_eq!(candidate.preflight(), preflight);
            let Some(offset) = candidate.exact_line_image_offset() else {
                unreachable!("matched exact line image");
            };
            assert_eq!(offset.control_point_count(), 3);
            assert_eq!(offset.distance(), &HReal::from(-1_i8));
            assert_eq!(candidate.distance(), &HReal::from(-1_i8));
        }
        relation => panic!("line-like quadratic staged right offset was not exact: {relation:?}"),
    }
    let rational_line = HRationalQuadraticBezier2::try_unit_end_weights(
        h_point_i32(-2, 0),
        h_point_i32(0, 0),
        h_point_i32(2, 0),
        HReal::from(2_i8),
    )
    .unwrap();
    let HClassification::Decided(preflight) = rational_line.offset_preflight(&policy) else {
        panic!("line-like rational conic offset preflight should be decided");
    };
    assert!(preflight.is_clear());
    match rational_line
        .offset_right_staged(HReal::one(), &policy)
        .unwrap()
    {
        HClassification::Decided(candidate) => {
            let HBezierOffsetCandidate2::ExactLineImage { preflight, .. } = &candidate else {
                panic!("line-like rational staged right offset was not exact: {candidate:?}");
            };
            assert!(preflight.is_clear());
            assert_eq!(candidate.preflight(), preflight);
            let Some(offset) = candidate.exact_line_image_offset() else {
                unreachable!("matched exact line image");
            };
            assert_eq!(offset.control_point_count(), 3);
            assert_eq!(offset.distance(), &HReal::from(-1_i8));
            assert_eq!(candidate.distance(), &HReal::from(-1_i8));
        }
        relation => panic!("line-like rational staged right offset was not exact: {relation:?}"),
    }
    let mixed_rational = HRationalQuadraticBezier2::try_unit_end_weights(
        h_point_i32(-2, 0),
        h_point_i32(0, 4),
        h_point_i32(2, 0),
        HReal::from(-1_i8),
    )
    .unwrap();
    let HClassification::Decided(preflight) = mixed_rational.offset_preflight(&policy) else {
        panic!("mixed-sign rational conic offset preflight should be decided");
    };
    assert!(
        preflight
            .risks()
            .contains(&HBezierOffsetRisk::ProjectiveDenominatorBoundary)
    );
    let collapsed_rational = HRationalQuadraticBezier2::try_unit_end_weights(
        h_point_i32(3, 4),
        h_point_i32(3, 4),
        h_point_i32(3, 4),
        HReal::from(2_i8),
    )
    .unwrap();
    match collapsed_rational
        .offset_left_staged(HReal::one(), &policy)
        .unwrap()
    {
        HClassification::Decided(HBezierOffsetCandidate2::Unresolved {
            preflight,
            distance,
        }) => {
            assert!(
                preflight
                    .risks()
                    .contains(&HBezierOffsetRisk::DegeneratePoint)
            );
            assert_eq!(distance, HReal::one());
        }
        relation => panic!("collapsed rational staged offset lost preflight: {relation:?}"),
    }
    let HClassification::Decided(line_polyline) = line_curve.flatten_certified(&options, &policy)
    else {
        panic!("line-like quadratic should flatten under generous fuzz options");
    };
    let simplified_line = match line_polyline.simplify_exact_collinear(&policy) {
        HClassification::Decided(polyline) => polyline,
        HClassification::Uncertain(reason) => {
            panic!("line-like quadratic simplification became uncertain: {reason:?}");
        }
    };
    let simplification = simplified_line
        .simplification_certificate()
        .expect("line-like simplification should carry a certificate");
    assert_eq!(simplification.source_start(), 0);
    assert_eq!(simplification.source_end(), line_polyline.points().len());
    assert_eq!(
        simplification.retained_vertex_count(),
        simplified_line.points().len()
    );
    assert_eq!(
        simplification.removed_vertex_count(),
        line_polyline.points().len() - simplified_line.points().len()
    );
    assert_eq!(simplification.error_bound(), &HReal::zero());
    assert_eq!(
        simplification.source_flattening_error(),
        line_polyline.certificate().max_error()
    );
    assert_eq!(
        simplification.source_flattening_max_depth(),
        options.max_depth()
    );
    assert_eq!(simplification.construction_policy(), &policy);
    assert_eq!(
        simplification.metric(),
        HBezierSimplificationErrorMetric::ExactPolylineImageDistance
    );
    assert_eq!(
        simplification.bound_kind(),
        HBezierSimplificationBoundKind::ProvenExact
    );
    match simplified_line
        .checked_offset_left(HReal::one(), &policy)
        .unwrap()
    {
        HClassification::Decided(offset) => {
            assert_eq!(offset.curve().segments().len(), 1);
            assert_eq!(offset.source_certificate(), simplified_line.certificate());
            assert_eq!(offset.distance(), &HReal::one());
        }
        relation => panic!("certified flattened line offset was not checked: {relation:?}"),
    }
    let right_preview = simplified_line.display_offset_right(HReal::one()).unwrap();
    assert_eq!(
        right_preview.segments().len(),
        simplified_line.points().len() - 1
    );
    assert_eq!(
        right_preview.source_certificate(),
        simplified_line.certificate()
    );
    assert_eq!(right_preview.distance(), &HReal::from(-1_i8));
    match simplified_line
        .checked_offset_right(HReal::one(), &policy)
        .unwrap()
    {
        HClassification::Decided(offset) => {
            assert_eq!(offset.curve().segments().len(), 1);
            assert_eq!(offset.source_certificate(), simplified_line.certificate());
            assert_eq!(offset.distance(), &HReal::from(-1_i8));
        }
        relation => panic!("certified flattened right offset was not checked: {relation:?}"),
    }
    match simplified_line.fit_exact_line(&policy).unwrap() {
        HClassification::Decided(HBezierLineFitRelation::Fit(fit)) => {
            let offset = fit.offset_left_exact(HReal::one()).unwrap();
            assert_eq!(offset.fit_certificate(), fit.fit_certificate());
            assert_eq!(offset.source_certificate(), fit.source_certificate());
            assert_eq!(offset.distance(), &HReal::one());
            let right_offset = fit.offset_right_exact(HReal::one()).unwrap();
            assert_eq!(right_offset.fit_certificate(), fit.fit_certificate());
            assert_eq!(right_offset.source_certificate(), fit.source_certificate());
            assert_eq!(right_offset.distance(), &HReal::from(-1_i8));
            assert_eq!(
                fit.fit_certificate().source_flattening_max_depth(),
                Some(options.max_depth())
            );
            assert_eq!(fit.fit_certificate().construction_policy(), &policy);
        }
        relation => panic!("certified flattened line did not fit exactly: {relation:?}"),
    }

    let linear_cubic = HCubicBezier2::new(
        h_point_i32(0, 0),
        h_point_i32(3, 0),
        h_point_i32(6, 0),
        h_point_i32(9, 0),
    );
    let one_third = (HReal::one() / HReal::from(3_i8)).unwrap();
    match linear_cubic
        .inverse_length_parameter_region(HReal::from(3_i8), 0, 0, &policy)
        .unwrap()
    {
        HClassification::Decided(region) => {
            assert_eq!(region.parameter_span().start(), &one_third);
            assert_eq!(region.parameter_span().end(), &one_third);
            assert_eq!(
                region.prefix_bounds_at_span_end().lower(),
                &HReal::from(3_i8)
            );
            assert_eq!(
                region.prefix_bounds_at_span_end().upper(),
                &HReal::from(3_i8)
            );
        }
        relation => panic!("linear cubic inverse length did not certify exactly: {relation:?}"),
    }
}

fn h_assert_bezier_boolean_fragment_locator_inputs(reader: &mut ByteReader<'_>) {
    let first_count = reader.usize_range(1, 6);
    let second_count = reader.usize_range(1, 6);
    let mut steps = Vec::with_capacity(first_count + second_count);
    let mut first_points = Vec::with_capacity(first_count);
    let mut second_points = Vec::with_capacity(second_count);
    for index in 0..first_count {
        first_points.push(h_point_i32(index as i32, reader.i32_range(-8, 8)));
        steps.push(HBezierBooleanTraversalStep2 {
            operand: HBezierBooleanTraversalOperand::First,
            fragment_index: index,
        });
    }
    for index in 0..second_count {
        second_points.push(h_point_i32(reader.i32_range(-8, 8), index as i32));
        steps.push(HBezierBooleanTraversalStep2 {
            operand: HBezierBooleanTraversalOperand::Second,
            fragment_index: index,
        });
    }
    let schedule = HBezierBooleanTraversalScheduleReport2 {
        status: HBezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: HBezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: first_count,
        second_fragment_count: second_count,
        steps,
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let report = HBezierBooleanFragmentLocatorInputReport2::from_representative_points(
        &schedule,
        h_boolean_op(reader),
        &first_points,
        &second_points,
    );
    assert_eq!(
        report.status,
        HBezierBooleanFragmentLocatorInputStatus::Ready
    );
    assert_eq!(report.input_count, schedule.steps.len());
    for (input, step) in report.inputs.iter().zip(schedule.steps.iter()) {
        assert_eq!(&input.step, step);
    }

    let stale = HBezierBooleanFragmentLocatorInputReport2::from_representative_points(
        &schedule,
        h_boolean_op(reader),
        &first_points[..first_count - 1],
        &second_points,
    );
    assert_eq!(
        stale.status,
        HBezierBooleanFragmentLocatorInputStatus::MissingFirstFragment
    );
    assert!(stale.has_blockers());
}

fn h_assert_bezier_boolean_endpoint_successors(reader: &mut ByteReader<'_>) {
    let cycle_count = reader.usize_range(1, 3);
    let mut steps = Vec::with_capacity(cycle_count * 2);
    let mut endpoints = Vec::with_capacity(cycle_count * 2);
    for cycle in 0..cycle_count {
        let base = (cycle as i32) * 10;
        let first = steps.len();
        steps.push(HBezierBooleanTraversalStep2 {
            operand: HBezierBooleanTraversalOperand::First,
            fragment_index: first,
        });
        endpoints.push((h_point_i32(base, 0), h_point_i32(base + 1, 0)));
        let second = steps.len();
        steps.push(HBezierBooleanTraversalStep2 {
            operand: HBezierBooleanTraversalOperand::First,
            fragment_index: second,
        });
        endpoints.push((h_point_i32(base + 1, 0), h_point_i32(base, 0)));
    }
    let schedule = HBezierBooleanTraversalScheduleReport2 {
        status: HBezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: HBezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: endpoints.len(),
        second_fragment_count: 0,
        steps,
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let ownership_facts = schedule
        .steps
        .iter()
        .map(|step| HBezierBooleanOwnershipFact2 {
            step: step.clone(),
            opposite_location: HBezierBooleanFragmentOwnershipLocation::Outside,
        })
        .collect::<Vec<_>>();
    let depth_facts = (0..cycle_count)
        .map(|loop_index| HBezierBooleanLoopNestingDepthFact2 {
            loop_index,
            nesting_depth: loop_index,
        })
        .collect::<Vec<_>>();
    let result = HBezierBooleanResultReport2::from_schedule_endpoint_successor_depth_facts(
        &schedule,
        h_boolean_op(reader),
        &ownership_facts,
        &endpoints,
        &[],
        cycle_count,
        0,
        &depth_facts,
    );
    assert_eq!(result.status, HBezierBooleanResultStatus::Ready);
    assert_eq!(result.assigned_loop_count, cycle_count);

    let blocked = HBezierBooleanResultReport2::from_schedule_endpoint_successor_depth_facts(
        &schedule,
        h_boolean_op(reader),
        &ownership_facts,
        &endpoints[..endpoints.len() - 1],
        &[],
        cycle_count,
        0,
        &depth_facts,
    );
    assert!(blocked.has_blockers());
}

fn h_endpoint_tangents(start: HPoint2, end: HPoint2) -> HBezierBooleanFragmentEndpointTangents2 {
    let tangent = HPoint2::new(end.x() - start.x(), end.y() - start.y());
    HBezierBooleanFragmentEndpointTangents2 {
        start,
        end,
        start_tangent: tangent.clone(),
        end_tangent: tangent,
    }
}

fn h_assert_bezier_boolean_tangent_branch_successors(_reader: &mut ByteReader<'_>) {
    let endpoints = [
        (h_point_i32(-1, 0), h_point_i32(0, 0)),
        (h_point_i32(0, 0), h_point_i32(0, -1)),
        (h_point_i32(0, -1), h_point_i32(-1, 0)),
        (h_point_i32(1, 0), h_point_i32(0, 0)),
        (h_point_i32(0, 0), h_point_i32(0, 1)),
        (h_point_i32(0, 1), h_point_i32(1, 0)),
    ];
    let tangents = endpoints
        .iter()
        .cloned()
        .map(|(start, end)| h_endpoint_tangents(start, end))
        .collect::<Vec<_>>();
    let plan = HBezierBooleanLoopAssemblyPlanReport2 {
        status: HBezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: HBezierBooleanAssemblyReadinessStatus::Ready,
        operation: HBooleanOp::Union,
        emitted_steps: (0..endpoints.len())
            .map(|fragment_index| HBezierBooleanOwnedTraversalStep2 {
                step: HBezierBooleanTraversalStep2 {
                    operand: HBezierBooleanTraversalOperand::First,
                    fragment_index,
                },
                opposite_location: HBezierBooleanFragmentOwnershipLocation::Outside,
                action: HBooleanFragmentAction::KeepSourceDirection,
            })
            .collect(),
        first_emitted_count: endpoints.len(),
        second_emitted_count: 0,
        keep_source_count: endpoints.len(),
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let traversal =
        HBezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(&plan, 1, 0);
    let branch_walk = HBezierBooleanLoopGraphMultiCycleWalkReport2::from_fragment_endpoint_tangents(
        &traversal,
        &plan,
        &tangents,
        &[],
        HBezierBooleanTangentTurnPolicy::CounterClockwise,
        &HCurvePolicy::certified(),
    );
    assert_eq!(
        branch_walk.status,
        HBezierBooleanLoopGraphMultiCycleWalkStatus::Ready
    );
    assert_eq!(branch_walk.cycle_step_counts, vec![3, 3]);
}

fn h_assert_bezier_boolean_overlap_bridge_successors(_reader: &mut ByteReader<'_>) {
    let endpoints = [
        (h_point_i32(0, 0), h_point_i32(1, 0)),
        (h_point_i32(1, 0), h_point_i32(0, 0)),
        (h_point_i32(0, 0), h_point_i32(1, 0)),
        (h_point_i32(1, 0), h_point_i32(0, 0)),
    ];
    let tangents = endpoints
        .iter()
        .cloned()
        .map(|(start, end)| h_endpoint_tangents(start, end))
        .collect::<Vec<_>>();
    let HClassification::Decided(overlaps) =
        HBezierBooleanOverlapResolutionReport2::from_overlap_events(
            &[HBezierBooleanOverlapEvent2 {
                first_range: HParamRange::new(HReal::zero(), HReal::one()),
                second_range: HParamRange::new(HReal::zero(), HReal::one()),
            }],
            &HCurvePolicy::certified(),
        )
    else {
        return;
    };
    let bridges = [
        HBezierBooleanOverlapBridgeFact2 {
            overlap_event_index: 0,
            successor: HBezierBooleanLoopGraphSuccessorFact2 {
                from_step_index: 0,
                to_step_index: 3,
            },
        },
        HBezierBooleanOverlapBridgeFact2 {
            overlap_event_index: 0,
            successor: HBezierBooleanLoopGraphSuccessorFact2 {
                from_step_index: 3,
                to_step_index: 0,
            },
        },
        HBezierBooleanOverlapBridgeFact2 {
            overlap_event_index: 0,
            successor: HBezierBooleanLoopGraphSuccessorFact2 {
                from_step_index: 1,
                to_step_index: 2,
            },
        },
        HBezierBooleanOverlapBridgeFact2 {
            overlap_event_index: 0,
            successor: HBezierBooleanLoopGraphSuccessorFact2 {
                from_step_index: 2,
                to_step_index: 1,
            },
        },
    ];
    let plan = HBezierBooleanLoopAssemblyPlanReport2 {
        status: HBezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: HBezierBooleanAssemblyReadinessStatus::Ready,
        operation: HBooleanOp::Union,
        emitted_steps: (0..endpoints.len())
            .map(|fragment_index| HBezierBooleanOwnedTraversalStep2 {
                step: HBezierBooleanTraversalStep2 {
                    operand: HBezierBooleanTraversalOperand::First,
                    fragment_index,
                },
                opposite_location: HBezierBooleanFragmentOwnershipLocation::Outside,
                action: HBooleanFragmentAction::KeepSourceDirection,
            })
            .collect(),
        first_emitted_count: endpoints.len(),
        second_emitted_count: 0,
        keep_source_count: endpoints.len(),
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let traversal =
        HBezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(&plan, 0, 1);
    let bridged =
        HBezierBooleanLoopGraphMultiCycleWalkReport2::from_fragment_endpoint_tangents_and_overlap_bridges(
            &traversal,
            &plan,
            &overlaps,
            &tangents,
            &[],
            &bridges,
            HBezierBooleanTangentTurnPolicy::CounterClockwise,
            &HCurvePolicy::certified(),
        );
    assert_eq!(
        bridged.status,
        HBezierBooleanLoopGraphMultiCycleWalkStatus::Ready
    );
    assert_eq!(bridged.cycle_step_counts, vec![2, 2]);
}

fn h_assert_bezier_boolean_linear_loop_locator(_reader: &mut ByteReader<'_>) {
    let endpoints = [
        (h_point_i32(0, 0), h_point_i32(10, 0)),
        (h_point_i32(10, 0), h_point_i32(10, 10)),
        (h_point_i32(10, 10), h_point_i32(0, 10)),
        (h_point_i32(0, 10), h_point_i32(0, 0)),
        (h_point_i32(2, 2), h_point_i32(4, 2)),
        (h_point_i32(4, 2), h_point_i32(4, 4)),
        (h_point_i32(4, 4), h_point_i32(2, 4)),
        (h_point_i32(2, 4), h_point_i32(2, 2)),
    ];
    let plan = HBezierBooleanLoopAssemblyPlanReport2 {
        status: HBezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: HBezierBooleanAssemblyReadinessStatus::Ready,
        operation: HBooleanOp::Union,
        emitted_steps: (0..endpoints.len())
            .map(|fragment_index| HBezierBooleanOwnedTraversalStep2 {
                step: HBezierBooleanTraversalStep2 {
                    operand: HBezierBooleanTraversalOperand::First,
                    fragment_index,
                },
                opposite_location: HBezierBooleanFragmentOwnershipLocation::Outside,
                action: HBooleanFragmentAction::KeepSourceDirection,
            })
            .collect(),
        first_emitted_count: endpoints.len(),
        second_emitted_count: 0,
        keep_source_count: endpoints.len(),
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = HBezierBooleanLoopClosureReport2::from_fragment_endpoints(&plan, &endpoints, &[]);
    let output = HBezierBooleanOutputLoopReport2::from_loop_closure(&closure);
    let flags = vec![true; output.directed_fragment_count];
    let replay = HBezierBooleanLoopContainmentQueryResultReport2::from_output_loop_linear_fragments(
        &output,
        &flags,
        &HCurvePolicy::certified(),
    );

    assert_eq!(
        replay.status,
        HBezierBooleanLoopContainmentQueryResultStatus::Ready
    );
    assert_eq!(replay.contains_count, 1);
    assert_eq!(replay.outside_count, 1);
    let certification =
        HBezierBooleanLoopContainmentCertificationReport2::from_output_loop_linear_fragments(
            &output,
            &flags,
            &HCurvePolicy::certified(),
        );
    assert_eq!(
        certification.status,
        HBezierBooleanLoopContainmentCertificationStatus::Ready
    );
    assert_eq!(certification.containment_facts, replay.containment_facts);
}

fn h_assert_bezier_boolean_quadratic_loop_locator(_reader: &mut ByteReader<'_>) {
    let fragments = vec![
        HQuadraticBezier2::new(h_point_i32(0, 0), h_point_i32(5, -2), h_point_i32(10, 0)),
        HQuadraticBezier2::new(h_point_i32(10, 0), h_point_i32(12, 5), h_point_i32(10, 10)),
        HQuadraticBezier2::new(h_point_i32(10, 10), h_point_i32(5, 12), h_point_i32(0, 10)),
        HQuadraticBezier2::new(h_point_i32(0, 10), h_point_i32(-2, 5), h_point_i32(0, 0)),
        HQuadraticBezier2::new(h_point_i32(2, 2), h_point_i32(3, 1), h_point_i32(4, 2)),
        HQuadraticBezier2::new(h_point_i32(4, 2), h_point_i32(5, 3), h_point_i32(4, 4)),
        HQuadraticBezier2::new(h_point_i32(4, 4), h_point_i32(3, 5), h_point_i32(2, 4)),
        HQuadraticBezier2::new(h_point_i32(2, 4), h_point_i32(1, 3), h_point_i32(2, 2)),
    ];
    let endpoints = fragments
        .iter()
        .map(|fragment| (fragment.start().clone(), fragment.end().clone()))
        .collect::<Vec<_>>();
    let plan = HBezierBooleanLoopAssemblyPlanReport2 {
        status: HBezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: HBezierBooleanAssemblyReadinessStatus::Ready,
        operation: HBooleanOp::Union,
        emitted_steps: (0..endpoints.len())
            .map(|fragment_index| HBezierBooleanOwnedTraversalStep2 {
                step: HBezierBooleanTraversalStep2 {
                    operand: HBezierBooleanTraversalOperand::First,
                    fragment_index,
                },
                opposite_location: HBezierBooleanFragmentOwnershipLocation::Outside,
                action: HBooleanFragmentAction::KeepSourceDirection,
            })
            .collect(),
        first_emitted_count: endpoints.len(),
        second_emitted_count: 0,
        keep_source_count: endpoints.len(),
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = HBezierBooleanLoopClosureReport2::from_fragment_endpoints(&plan, &endpoints, &[]);
    let output = HBezierBooleanOutputLoopReport2::from_loop_closure(&closure);
    let first = HBezierBooleanQuadraticFragmentReport2 {
        status: HBezierBooleanFragmentConstructionStatus::Ready,
        readiness_status: HBezierBooleanConstructionReadinessStatus::Ready,
        source_parameter_count: 0,
        endpoint_parameter_count: 0,
        out_of_range_parameter_count: 0,
        inserted_parameter_count: 0,
        inserted_parameters: Vec::new(),
        fragments,
    };
    let second = HBezierBooleanQuadraticFragmentReport2 {
        fragments: Vec::new(),
        ..first.clone()
    };
    let replay =
        HBezierBooleanLoopContainmentQueryResultReport2::from_output_loop_quadratic_fragments(
            &output,
            &first,
            &second,
            &HCurvePolicy::certified(),
        );

    assert_eq!(
        replay.status,
        HBezierBooleanLoopContainmentQueryResultStatus::Ready
    );
    assert_eq!(replay.contains_count, 1);
    assert_eq!(replay.outside_count, 1);
}

fn h_assert_bezier_boolean_cubic_loop_locator(_reader: &mut ByteReader<'_>) {
    let fragments = vec![
        HCubicBezier2::new(
            h_point_i32(0, 0),
            h_point_i32(3, -2),
            h_point_i32(7, -2),
            h_point_i32(10, 0),
        ),
        HCubicBezier2::new(
            h_point_i32(10, 0),
            h_point_i32(10, 3),
            h_point_i32(10, 7),
            h_point_i32(10, 10),
        ),
        HCubicBezier2::new(
            h_point_i32(10, 10),
            h_point_i32(7, 12),
            h_point_i32(3, 12),
            h_point_i32(0, 10),
        ),
        HCubicBezier2::new(
            h_point_i32(0, 10),
            h_point_i32(0, 7),
            h_point_i32(0, 3),
            h_point_i32(0, 0),
        ),
        HCubicBezier2::new(
            h_point_i32(2, 5),
            h_point_i32(3, 4),
            h_point_i32(3, 4),
            h_point_i32(4, 5),
        ),
        HCubicBezier2::new(
            h_point_i32(4, 5),
            h_point_i32(4, 6),
            h_point_i32(4, 6),
            h_point_i32(4, 7),
        ),
        HCubicBezier2::new(
            h_point_i32(4, 7),
            h_point_i32(3, 8),
            h_point_i32(3, 8),
            h_point_i32(2, 7),
        ),
        HCubicBezier2::new(
            h_point_i32(2, 7),
            h_point_i32(2, 6),
            h_point_i32(2, 6),
            h_point_i32(2, 5),
        ),
    ];
    let endpoints = fragments
        .iter()
        .map(|fragment| (fragment.start().clone(), fragment.end().clone()))
        .collect::<Vec<_>>();
    let plan = HBezierBooleanLoopAssemblyPlanReport2 {
        status: HBezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: HBezierBooleanAssemblyReadinessStatus::Ready,
        operation: HBooleanOp::Union,
        emitted_steps: (0..endpoints.len())
            .map(|fragment_index| HBezierBooleanOwnedTraversalStep2 {
                step: HBezierBooleanTraversalStep2 {
                    operand: HBezierBooleanTraversalOperand::First,
                    fragment_index,
                },
                opposite_location: HBezierBooleanFragmentOwnershipLocation::Outside,
                action: HBooleanFragmentAction::KeepSourceDirection,
            })
            .collect(),
        first_emitted_count: endpoints.len(),
        second_emitted_count: 0,
        keep_source_count: endpoints.len(),
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = HBezierBooleanLoopClosureReport2::from_fragment_endpoints(&plan, &endpoints, &[]);
    let output = HBezierBooleanOutputLoopReport2::from_loop_closure(&closure);
    let first = HBezierBooleanCubicFragmentReport2 {
        status: HBezierBooleanFragmentConstructionStatus::Ready,
        readiness_status: HBezierBooleanConstructionReadinessStatus::Ready,
        source_parameter_count: 0,
        endpoint_parameter_count: 0,
        out_of_range_parameter_count: 0,
        inserted_parameter_count: 0,
        inserted_parameters: Vec::new(),
        fragments,
    };
    let second = HBezierBooleanCubicFragmentReport2 {
        fragments: Vec::new(),
        ..first.clone()
    };
    let replay = HBezierBooleanLoopContainmentQueryResultReport2::from_output_loop_cubic_fragments(
        &output,
        &first,
        &second,
        &HCurvePolicy::certified(),
    );

    assert_eq!(
        replay.status,
        HBezierBooleanLoopContainmentQueryResultStatus::Ready
    );
    assert_eq!(replay.contains_count, 1);
    assert_eq!(replay.outside_count, 1);
    let certification =
        HBezierBooleanLoopContainmentCertificationReport2::from_output_loop_cubic_fragments(
            &output,
            &first,
            &second,
            &HCurvePolicy::certified(),
        );
    assert_eq!(
        certification.status,
        HBezierBooleanLoopContainmentCertificationStatus::Ready
    );
    assert_eq!(certification.containment_facts, replay.containment_facts);
}

fn h_assert_bezier_boolean_rational_quadratic_loop_locator(_reader: &mut ByteReader<'_>) {
    let fragments = vec![
        HRationalQuadraticBezier2::try_unit_end_weights(
            h_point_i32(0, 0),
            h_point_i32(5, -2),
            h_point_i32(10, 0),
            HReal::from(2_i8),
        )
        .unwrap(),
        HRationalQuadraticBezier2::try_unit_end_weights(
            h_point_i32(10, 0),
            h_point_i32(12, 5),
            h_point_i32(10, 10),
            HReal::one(),
        )
        .unwrap(),
        HRationalQuadraticBezier2::try_unit_end_weights(
            h_point_i32(10, 10),
            h_point_i32(5, 12),
            h_point_i32(0, 10),
            HReal::from(2_i8),
        )
        .unwrap(),
        HRationalQuadraticBezier2::try_unit_end_weights(
            h_point_i32(0, 10),
            h_point_i32(-2, 5),
            h_point_i32(0, 0),
            HReal::one(),
        )
        .unwrap(),
        HRationalQuadraticBezier2::try_unit_end_weights(
            h_point_i32(2, 5),
            h_point_i32(3, 4),
            h_point_i32(4, 5),
            HReal::from(2_i8),
        )
        .unwrap(),
        HRationalQuadraticBezier2::try_unit_end_weights(
            h_point_i32(4, 5),
            h_point_i32(5, 6),
            h_point_i32(4, 7),
            HReal::one(),
        )
        .unwrap(),
        HRationalQuadraticBezier2::try_unit_end_weights(
            h_point_i32(4, 7),
            h_point_i32(3, 8),
            h_point_i32(2, 7),
            HReal::from(2_i8),
        )
        .unwrap(),
        HRationalQuadraticBezier2::try_unit_end_weights(
            h_point_i32(2, 7),
            h_point_i32(1, 6),
            h_point_i32(2, 5),
            HReal::one(),
        )
        .unwrap(),
    ];
    let endpoints = fragments
        .iter()
        .map(|fragment| (fragment.start().clone(), fragment.end().clone()))
        .collect::<Vec<_>>();
    let plan = HBezierBooleanLoopAssemblyPlanReport2 {
        status: HBezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: HBezierBooleanAssemblyReadinessStatus::Ready,
        operation: HBooleanOp::Union,
        emitted_steps: (0..endpoints.len())
            .map(|fragment_index| HBezierBooleanOwnedTraversalStep2 {
                step: HBezierBooleanTraversalStep2 {
                    operand: HBezierBooleanTraversalOperand::First,
                    fragment_index,
                },
                opposite_location: HBezierBooleanFragmentOwnershipLocation::Outside,
                action: HBooleanFragmentAction::KeepSourceDirection,
            })
            .collect(),
        first_emitted_count: endpoints.len(),
        second_emitted_count: 0,
        keep_source_count: endpoints.len(),
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = HBezierBooleanLoopClosureReport2::from_fragment_endpoints(&plan, &endpoints, &[]);
    let output = HBezierBooleanOutputLoopReport2::from_loop_closure(&closure);
    let first = HBezierBooleanRationalQuadraticFragmentReport2 {
        status: HBezierBooleanFragmentConstructionStatus::Ready,
        readiness_status: HBezierBooleanConstructionReadinessStatus::Ready,
        source_parameter_count: 0,
        endpoint_parameter_count: 0,
        out_of_range_parameter_count: 0,
        inserted_parameter_count: 0,
        inserted_parameters: Vec::new(),
        fragments,
    };
    let second = HBezierBooleanRationalQuadraticFragmentReport2 {
        fragments: Vec::new(),
        ..first.clone()
    };
    let replay = HBezierBooleanLoopContainmentQueryResultReport2::from_output_loop_rational_quadratic_fragments(
        &output,
        &first,
        &second,
        &HCurvePolicy::certified(),
    );

    assert_eq!(
        replay.status,
        HBezierBooleanLoopContainmentQueryResultStatus::Ready
    );
    assert_eq!(replay.contains_count, 1);
    assert_eq!(replay.outside_count, 1);
}

/// Aggregate hypercurve fuzz entrypoint covering public APIs and cross-path invariants.
fn h_algebraic_root_report(parameter: HReal, exact: bool) -> HAlgebraicRootRepresentationReport {
    let lower = parameter.clone() - h_ratio(1, 128);
    let upper = parameter.clone() + h_ratio(1, 128);
    HAlgebraicRootRepresentationReport {
        constraint_index: 0,
        symbol: Some(HSymbolId(0)),
        status: HAlgebraicRootRepresentationStatus::Represented,
        roots: vec![HAlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: HSymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-parameter.clone(), HReal::one()],
            interval: HIsolatedRootInterval {
                lower,
                upper,
                exact_root: exact.then_some(parameter),
                distinct_root_count: 1,
            },
            kind: if exact {
                HAlgebraicRootKind::ExactRationalWitness
            } else {
                HAlgebraicRootKind::IsolatingInterval
            },
            validation: HAlgebraicRootValidationReport {
                status: HAlgebraicRootValidationStatus::Valid,
                message: None,
            },
        }],
        message: None,
    }
}

/// Fuzzes the algebraic carrier boundary without running the heavy libFuzzer target here.
///
/// The invariant is Yap-style: ordered interval-only roots are valid carrier
/// evidence, but they must not be advertised as lowerable by the rational split
/// bridge until an algebraic splitter exists.
pub fn h_assert_bezier_boolean_algebraic_parameter_carrier(reader: &mut ByteReader<'_>) {
    let exact = reader.bool();
    let numerator = reader.i32_range(1, 7);
    let denominator = reader.i32_range(8, 16);
    let parameter = h_ratio(numerator, denominator);
    let range = HBezierPathRangeOrderReport2::from_graph_order(
        &HBezierMonotoneGraphOrder::IntersectsOrTouches {
            parameters: Vec::new(),
            spans: vec![HBezierMonotoneSpan::new(h_ratio(1, 8), h_ratio(7, 8))],
        },
    );
    let scheduler = HBezierBooleanPathSchedulerReport2::from_batches(
        HBezierBooleanBatchHandoffReport2::from_handoff_reports(&[]),
        HBezierPathRangeBatchReport2::from_range_reports(&[range]),
    );
    let handoff =
        match HBezierBooleanAlgebraicParameterHandoffReport2::from_hypersolve_algebraic_root_reports(
            &scheduler,
            &[h_algebraic_root_report(parameter, exact)],
            &h_policy(),
        ) {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(_) => return,
        };
    let readiness =
        match HBezierBooleanAlgebraicParameterReadinessReport2::from_handoff(&handoff, &h_policy())
        {
            HClassification::Decided(report) => report,
            HClassification::Uncertain(_) => return,
        };
    let mut ordering = HBezierBooleanAlgebraicParameterOrderingReport2::from_readiness(
        &readiness,
        HAlgebraicRootRefinementComparisonConfig::default(),
        &h_policy(),
    );
    if ordering.status == HBezierBooleanAlgebraicParameterOrderingStatus::Ready {
        let role = match reader.byte() % 3 {
            0 => HBezierBooleanAlgebraicParameterRole::FirstCurve,
            1 => HBezierBooleanAlgebraicParameterRole::SecondCurve,
            _ => HBezierBooleanAlgebraicParameterRole::SharedRange,
        };
        ordering.sorted_events[0].role = role;
    }
    let carrier = HBezierBooleanAlgebraicParameterCarrierReport2::from_ordering(&ordering);
    let bridge = HBezierBooleanAlgebraicSplitBridgeReport2::from_ordering(&ordering, &h_policy());

    assert_eq!(carrier.ordering_status, ordering.status);
    if ordering.status == HBezierBooleanAlgebraicParameterOrderingStatus::Ready {
        assert_eq!(
            carrier.status,
            HBezierBooleanAlgebraicParameterCarrierStatus::Ready
        );
        assert_eq!(carrier.ordered_event_count, 1);
        assert_eq!(carrier.exact_rational_event_count, usize::from(exact));
        assert_eq!(carrier.interval_event_count, usize::from(!exact));
        assert_eq!(
            carrier.first_curve_events.len()
                + carrier.second_curve_events.len()
                + carrier.shared_range_events.len(),
            1
        );
        assert_eq!(carrier.can_feed_rational_split_bridge(), exact);
        if let HClassification::Decided(bridge) = bridge {
            assert_eq!(bridge.is_ready(), exact);
            assert_eq!(bridge.exact_rational_parameter_count, usize::from(exact));
            assert_eq!(bridge.non_rational_parameter_count, usize::from(!exact));
        }
    } else {
        assert!(carrier.has_blockers() || carrier.ordered_event_count == 0);
    }
}

pub fn h_assert_full_api(reader: &mut ByteReader<'_>) {
    match reader.byte() % 24 {
        0 => h_assert_segment_intersections(reader),
        1 => h_assert_segment_containment_and_reversal(reader),
        2 => h_assert_contour_region_classification(reader),
        3 => h_assert_region_boolean(reader),
        4 => h_assert_events_and_fragments(reader),
        5 => h_assert_bboxes_curve_strings_and_splits(reader),
        6 => h_assert_offsets_and_self_contacts(reader),
        7 => h_assert_boundary_nesting(reader),
        8 => h_assert_boolean_boundary_pipeline(reader),
        9 => h_assert_offset_cap_matrix(reader),
        10 => h_assert_polyline_reconstruction(reader),
        11 => h_assert_bezier_conic_relations(reader),
        12 => h_assert_bezier_point_image_fits(reader),
        13 => h_assert_region_boolean_antagonistic_contract(reader),
        14 => h_assert_bezier_boolean_fragment_locator_inputs(reader),
        15 => h_assert_bezier_boolean_endpoint_successors(reader),
        16 => h_assert_bezier_boolean_tangent_branch_successors(reader),
        17 => h_assert_bezier_boolean_overlap_bridge_successors(reader),
        18 => h_assert_bezier_boolean_linear_loop_locator(reader),
        19 => h_assert_bezier_boolean_quadratic_loop_locator(reader),
        20 => h_assert_bezier_boolean_rational_quadratic_loop_locator(reader),
        21 => h_assert_bezier_boolean_cubic_loop_locator(reader),
        22 => h_assert_bezier_boolean_algebraic_parameter_carrier(reader),
        _ => h_assert_adversarial_polygon_pipeline(reader),
    }
}
