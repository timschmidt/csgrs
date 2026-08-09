//! Contour nesting and material/hole role assignment.
//!
//! This module turns already-closed native boundary contours into private
//! material/hole bins. It assumes intersections and overlaps have already been
//! resolved by earlier topology stages.
use std::{cmp::Ordering, sync::OnceLock};

use hyperreal::Real;

use crate::bbox::{
    Aabb2, aabb_decided_misses_point, aabbs_decided_disjoint, decided_contour_aabb,
    decided_segment_aabb,
};
use crate::bezier_region::CurveRegionArrangementStage2;
use crate::classify::compare_reals;
use crate::region::LineArcRegion2;
use crate::{
    ArcArcIntersection, CircularArc2, Classification, Contour2, ContourPointLocation, CurveContext,
    CurveError, CurveResult, FillRule, LineArcIntersection, LineArcOrder, LineLineIntersection,
    LineSeg2, Point2, RetainedTopologyStatus, Segment2, SegmentIntersection, SegmentKindCounts,
    UncertaintyReason,
};

/// Private result of arranging unordered native boundaries for promotion into
/// the authoritative [`crate::CurveRegion2`].
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct RegionArrangement2 {
    pub(crate) fill_rule: FillRule,
    pub(crate) source_segment_count: usize,
    pub(crate) stage: CurveRegionArrangementStage2,
    pub(crate) status: RetainedTopologyStatus,
    pub(crate) blocker: Option<UncertaintyReason>,
    pub(crate) output_ring_count: Option<usize>,
    pub(crate) output_boundary_segment_count: Option<usize>,
    pub(crate) output_boundary_segment_kind_counts: Option<SegmentKindCounts>,
    pub(crate) region: Option<LineArcRegion2>,
}

/// Compact terminal facts for unordered exact segment region construction.
#[derive(Clone, Debug, PartialEq)]
struct RegionLineSegmentRegionBuildEvidence2 {
    stage: CurveRegionArrangementStage2,
    output_ring_count: Option<usize>,
    output_boundary_segment_count: Option<usize>,
    output_boundary_segment_kind_counts: Option<SegmentKindCounts>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Internal staging result for unordered exact segment region construction.
#[derive(Clone, Debug, PartialEq)]
struct RegionLineSegmentRegionBuildResult2 {
    region: Option<LineArcRegion2>,
    evidence: RegionLineSegmentRegionBuildEvidence2,
}

#[derive(Clone, Debug, PartialEq)]
struct BoundaryContourNestingDepths {
    depths: Vec<usize>,
}

#[derive(Clone, Debug, PartialEq)]
enum BoundaryContourNestingOutcome {
    Decided(BoundaryContourNestingDepths),
    Blocked(UncertaintyReason),
}

fn evaluate_unordered_line_segments_region_result(
    segments: &[LineSeg2],
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
    if segments.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }

    let arranged = match arrange_line_segments_at_point_intersections(segments, policy)? {
        Ok(arranged) => arranged,
        Err(blocker) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                evidence: blocked_line_segment_region_evidence(
                    CurveRegionArrangementStage2::RingAssembly,
                    retained_status_for_line_segment_region_blocker(blocker),
                    blocker,
                ),
            });
        }
    };

    if let Err(blocker) = validate_arranged_line_endpoint_graph(&arranged.segments, policy) {
        return Ok(RegionLineSegmentRegionBuildResult2 {
            region: None,
            evidence: blocked_line_segment_region_evidence(
                CurveRegionArrangementStage2::RingAssembly,
                retained_status_for_line_segment_region_blocker(blocker),
                blocker,
            ),
        });
    }

    let rings = match assemble_unordered_line_segment_rings(&arranged.segments, policy) {
        Ok(rings) => rings,
        Err(blocker) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                evidence: blocked_line_segment_region_evidence(
                    CurveRegionArrangementStage2::RingAssembly,
                    retained_status_for_line_segment_region_blocker(blocker),
                    blocker,
                ),
            });
        }
    };

    let mut contours = Vec::with_capacity(rings.len());
    for ring in rings {
        let contour = Contour2::try_new_with_fill_rule(
            ring.into_iter().map(Segment2::Line).collect(),
            fill_rule,
        )?;
        contours.push(contour);
    }

    finish_nested_contours(contours, policy)
}

fn evaluate_unordered_segments_region_result(
    segments: &[Segment2],
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
    if segments.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }

    let arranged = match arrange_native_segments_at_point_intersections(segments, policy)? {
        Ok(arranged) => arranged,
        Err(blocker) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                evidence: blocked_line_segment_region_evidence(
                    CurveRegionArrangementStage2::RingAssembly,
                    retained_status_for_line_segment_region_blocker(blocker),
                    blocker,
                ),
            });
        }
    };

    if let Err(blocker) = validate_arranged_native_endpoint_graph(&arranged.segments, policy) {
        return Ok(RegionLineSegmentRegionBuildResult2 {
            region: None,
            evidence: blocked_line_segment_region_evidence(
                CurveRegionArrangementStage2::RingAssembly,
                retained_status_for_line_segment_region_blocker(blocker),
                blocker,
            ),
        });
    }

    let rings = match assemble_unordered_native_segment_rings(&arranged.segments, policy) {
        Ok(rings) => rings,
        Err(blocker) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                evidence: blocked_line_segment_region_evidence(
                    CurveRegionArrangementStage2::RingAssembly,
                    retained_status_for_line_segment_region_blocker(blocker),
                    blocker,
                ),
            });
        }
    };

    let mut contours = Vec::with_capacity(rings.len());
    for ring in rings {
        contours.push(Contour2::try_new_with_fill_rule(ring, fill_rule)?);
    }

    finish_nested_contours(contours, policy)
}

fn finish_nested_contours(
    contours: Vec<Contour2>,
    policy: &CurveContext,
) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
    let (region, status, blocker) = match LineArcRegion2::from_boundary_contours(contours, policy)?
    {
        Classification::Decided(region) => {
            (Some(region), RetainedTopologyStatus::NativeExact, None)
        }
        Classification::Uncertain(reason) => (
            None,
            retained_status_for_boundary_contour_blocker(reason),
            Some(reason),
        ),
    };
    let output_ring_count = region
        .as_ref()
        .map(|region| region.material_contours().len() + region.hole_contours().len());
    let output_boundary_segment_count = region.as_ref().map(|region| {
        region
            .material_contours()
            .iter()
            .chain(region.hole_contours())
            .map(|contour| contour.segments().len())
            .sum()
    });
    let output_boundary_segment_kind_counts = region.as_ref().map(region_segment_kind_counts);
    Ok(RegionLineSegmentRegionBuildResult2 {
        region,
        evidence: RegionLineSegmentRegionBuildEvidence2 {
            stage: CurveRegionArrangementStage2::RegionRoleAssignment,
            output_ring_count,
            output_boundary_segment_count,
            output_boundary_segment_kind_counts,
            status,
            blocker,
        },
    })
}

impl LineArcRegion2 {
    /// Arranges unordered exact line/arc segments into a retained region result.
    pub(crate) fn arrange_unordered_segments(
        source_segments: Vec<Segment2>,
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> CurveResult<RegionArrangement2> {
        let source_segment_count = source_segments.len();
        let staging_result =
            evaluate_unordered_segments_region_result(&source_segments, fill_rule, policy)?;
        Ok(finish_native_arrangement(
            staging_result,
            fill_rule,
            source_segment_count,
        ))
    }

    /// Arranges borrowed unordered exact line/arc segments into a retained region result.
    pub(crate) fn arrange_unordered_segments_borrowed(
        source_segments: &[Segment2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> CurveResult<RegionArrangement2> {
        let staging_result =
            evaluate_unordered_segments_region_result(source_segments, fill_rule, policy)?;
        Ok(finish_native_arrangement(
            staging_result,
            fill_rule,
            source_segments.len(),
        ))
    }

    /// Arranges unordered exact line segments using the specialized line pipeline.
    pub(crate) fn arrange_unordered_line_segments(
        source_segments: Vec<LineSeg2>,
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> CurveResult<RegionArrangement2> {
        let source_segment_count = source_segments.len();
        let staging_result =
            evaluate_unordered_line_segments_region_result(&source_segments, fill_rule, policy)?;
        Ok(finish_native_arrangement(
            staging_result,
            fill_rule,
            source_segment_count,
        ))
    }

    /// Arranges borrowed unordered exact lines using the specialized line pipeline.
    pub(crate) fn arrange_unordered_line_segments_borrowed(
        source_segments: &[LineSeg2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> CurveResult<RegionArrangement2> {
        let staging_result =
            evaluate_unordered_line_segments_region_result(source_segments, fill_rule, policy)?;
        Ok(finish_native_arrangement(
            staging_result,
            fill_rule,
            source_segments.len(),
        ))
    }

    /// Builds a region by nesting closed boundary contours and retaining role evidence.
    ///
    /// This is the evidence-bearing counterpart to
    /// Contours at even containment depth
    /// become material and odd-depth contours become holes. If intersections,
    /// touches, or undecided containment predicates prevent role assignment, no
    /// region is materialized and the evidence carries the blocker.
    pub(crate) fn from_boundary_contours(
        contours: Vec<Contour2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        Ok(match contour_nesting_depths(&contours, policy)? {
            BoundaryContourNestingOutcome::Decided(nesting) => {
                Classification::Decided(assign_boundary_contour_roles(contours, &nesting))
            }
            BoundaryContourNestingOutcome::Blocked(reason) => Classification::Uncertain(reason),
        })
    }
}

fn assign_boundary_contour_roles(
    contours: Vec<Contour2>,
    nesting: &BoundaryContourNestingDepths,
) -> LineArcRegion2 {
    let mut material_contours = Vec::new();
    let mut hole_contours = Vec::new();
    for (contour, depth) in contours.into_iter().zip(&nesting.depths) {
        if depth % 2 == 0 {
            material_contours.push(contour);
        } else {
            hole_contours.push(contour);
        }
    }
    LineArcRegion2::new(material_contours, hole_contours)
}

fn finish_native_arrangement(
    staging_result: RegionLineSegmentRegionBuildResult2,
    fill_rule: FillRule,
    source_segment_count: usize,
) -> RegionArrangement2 {
    let RegionLineSegmentRegionBuildResult2 { region, evidence } = staging_result;
    let RegionLineSegmentRegionBuildEvidence2 {
        stage,
        output_ring_count,
        output_boundary_segment_count,
        output_boundary_segment_kind_counts,
        status,
        blocker,
    } = evidence;
    RegionArrangement2 {
        fill_rule,
        source_segment_count,
        stage,
        status,
        blocker,
        output_ring_count,
        output_boundary_segment_count,
        output_boundary_segment_kind_counts,
        region,
    }
}

#[derive(Clone, Debug, PartialEq)]
struct ArrangedLineSegment {
    line: LineSeg2,
}

#[derive(Clone, Debug, PartialEq)]
struct ArrangedLineSegments {
    segments: Vec<ArrangedLineSegment>,
}

#[derive(Clone, Debug, PartialEq)]
struct ArrangedNativeSegment {
    segment: Segment2,
}

#[derive(Clone, Debug, PartialEq)]
struct ArrangedNativeSegments {
    segments: Vec<ArrangedNativeSegment>,
}

impl ArrangedLineSegment {
    fn reversed(&self) -> Self {
        Self {
            line: self.line.reversed(),
        }
    }
}

impl ArrangedNativeSegment {
    fn reversed(&self) -> Self {
        Self {
            segment: self.segment.reversed(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum EndpointCandidate {
    Start,
    End,
}

#[derive(Clone, Debug, PartialEq)]
struct LineSegmentSplitMarker {
    param: Real,
}

#[derive(Clone, Debug, PartialEq)]
struct NativeSegmentSplitMarker {
    param: Real,
    point: Point2,
}

fn validate_arranged_line_endpoint_graph(
    segments: &[ArrangedLineSegment],
    policy: &CurveContext,
) -> Result<(), UncertaintyReason> {
    let endpoints = segments
        .iter()
        .enumerate()
        .flat_map(|(segment_index, segment)| {
            [
                (segment_index, segment.line.start()),
                (segment_index, segment.line.end()),
            ]
        })
        .collect::<Vec<_>>();
    validate_exact_endpoint_graph(&endpoints, policy)
}

fn validate_arranged_native_endpoint_graph(
    segments: &[ArrangedNativeSegment],
    policy: &CurveContext,
) -> Result<(), UncertaintyReason> {
    let endpoints = segments
        .iter()
        .enumerate()
        .flat_map(|(segment_index, segment)| {
            [
                (segment_index, segment.segment.start()),
                (segment_index, segment.segment.end()),
            ]
        })
        .collect::<Vec<_>>();
    validate_exact_endpoint_graph(&endpoints, policy)
}

fn validate_exact_endpoint_graph(
    endpoints: &[(usize, &Point2)],
    policy: &CurveContext,
) -> Result<(), UncertaintyReason> {
    for (endpoint_index, (segment_index, point)) in endpoints.iter().enumerate() {
        let mut exact_match_count = 0_usize;
        for (candidate_index, (candidate_segment_index, candidate)) in endpoints.iter().enumerate()
        {
            if endpoint_index == candidate_index || segment_index == candidate_segment_index {
                continue;
            }
            match exact_points_match(point, candidate, policy) {
                Classification::Decided(true) => exact_match_count += 1,
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Err(reason),
            }
        }
        if exact_match_count != 1 {
            return Err(UncertaintyReason::Boundary);
        }
    }
    Ok(())
}
fn arrange_line_segments_at_point_intersections(
    segments: &[LineSeg2],
    policy: &CurveContext,
) -> CurveResult<Result<ArrangedLineSegments, UncertaintyReason>> {
    let mut markers = segments
        .iter()
        .map(|_| {
            vec![
                LineSegmentSplitMarker {
                    param: Real::zero(),
                },
                LineSegmentSplitMarker { param: Real::one() },
            ]
        })
        .collect::<Vec<_>>();
    let segment_boxes = segments
        .iter()
        .map(|line| match Aabb2::from_line(line, policy) {
            Classification::Decided(bbox) => Some(bbox),
            Classification::Uncertain(_) => None,
        })
        .collect::<Vec<_>>();

    for (first_index, first) in segments.iter().enumerate() {
        for (second_offset, second) in segments[first_index + 1..].iter().enumerate() {
            let second_index = first_index + 1 + second_offset;
            if let (Some(first_box), Some(second_box)) =
                (&segment_boxes[first_index], &segment_boxes[second_index])
                && aabbs_decided_disjoint(first_box, second_box, policy)
            {
                continue;
            }
            match first.intersect_line(second, policy)? {
                LineLineIntersection::None => {}
                LineLineIntersection::Point {
                    a_param, b_param, ..
                } => {
                    if insert_line_split_marker(&mut markers[first_index], a_param, policy)
                        .is_none()
                        || insert_line_split_marker(&mut markers[second_index], b_param, policy)
                            .is_none()
                    {
                        return Ok(Err(UncertaintyReason::Ordering));
                    }
                }
                LineLineIntersection::Overlap { .. } => {
                    return Ok(Err(UncertaintyReason::Boundary));
                }
                LineLineIntersection::Uncertain { reason } => {
                    return Ok(Err(reason));
                }
            }
        }
    }

    let mut arranged = Vec::new();
    for (line, source_markers) in segments.iter().zip(markers.iter_mut()) {
        sort_line_split_markers(source_markers, policy).ok_or(CurveError::Topology(
            "line split markers could not be sorted".into(),
        ))?;
        for pair in source_markers.windows(2) {
            let start_param = pair[0].param.clone();
            let end_param = pair[1].param.clone();
            match compare_reals(&start_param, &end_param, policy) {
                Some(Ordering::Less) => {
                    arranged.push(ArrangedLineSegment {
                        line: LineSeg2::try_new(
                            line.point_at(start_param),
                            line.point_at(end_param),
                        )?,
                    });
                }
                Some(Ordering::Equal) => {}
                Some(Ordering::Greater) | None => {
                    return Ok(Err(UncertaintyReason::Ordering));
                }
            }
        }
    }

    Ok(Ok(ArrangedLineSegments { segments: arranged }))
}

fn insert_line_split_marker(
    markers: &mut Vec<LineSegmentSplitMarker>,
    param: Real,
    policy: &CurveContext,
) -> Option<()> {
    for marker in markers.iter() {
        if compare_reals(&marker.param, &param, policy)? == Ordering::Equal {
            return Some(());
        }
    }
    markers.push(LineSegmentSplitMarker { param });
    Some(())
}

fn sort_line_split_markers(
    markers: &mut [LineSegmentSplitMarker],
    policy: &CurveContext,
) -> Option<()> {
    let mut failed = false;
    markers.sort_by(|left, right| {
        compare_reals(&left.param, &right.param, policy).unwrap_or_else(|| {
            failed = true;
            Ordering::Equal
        })
    });
    (!failed).then_some(())
}

fn arrange_native_segments_at_point_intersections(
    segments: &[Segment2],
    policy: &CurveContext,
) -> CurveResult<Result<ArrangedNativeSegments, UncertaintyReason>> {
    let mut markers = segments
        .iter()
        .map(|segment| {
            vec![
                NativeSegmentSplitMarker {
                    param: Real::zero(),
                    point: segment.start().clone(),
                },
                NativeSegmentSplitMarker {
                    param: Real::one(),
                    point: segment.end().clone(),
                },
            ]
        })
        .collect::<Vec<_>>();
    let segment_boxes = segments
        .iter()
        .map(|segment| match Aabb2::from_segment(segment, policy) {
            Ok(Classification::Decided(bbox)) => Some(bbox),
            Ok(Classification::Uncertain(_)) | Err(_) => None,
        })
        .collect::<Vec<_>>();

    for (first_index, first) in segments.iter().enumerate() {
        for (second_offset, second) in segments[first_index + 1..].iter().enumerate() {
            let second_index = first_index + 1 + second_offset;
            if let (Some(first_box), Some(second_box)) =
                (&segment_boxes[first_index], &segment_boxes[second_index])
                && aabbs_decided_disjoint(first_box, second_box, policy)
            {
                continue;
            }
            match native_segment_intersection_split_markers(first, second, policy)? {
                NativeSegmentIntersectionMarkers::None => {}
                NativeSegmentIntersectionMarkers::Points(points) => {
                    for point in points {
                        if insert_native_split_marker(
                            &mut markers[first_index],
                            NativeSegmentSplitMarker {
                                param: point.first_param,
                                point: point.point.clone(),
                            },
                            policy,
                        )
                        .is_none()
                            || insert_native_split_marker(
                                &mut markers[second_index],
                                NativeSegmentSplitMarker {
                                    param: point.second_param,
                                    point: point.point,
                                },
                                policy,
                            )
                            .is_none()
                        {
                            return Ok(Err(UncertaintyReason::Ordering));
                        }
                    }
                }
                NativeSegmentIntersectionMarkers::Overlap => {
                    return Ok(Err(UncertaintyReason::Boundary));
                }
                NativeSegmentIntersectionMarkers::Uncertain(reason) => {
                    return Ok(Err(reason));
                }
            }
        }
    }

    let mut arranged = Vec::new();
    for (segment, source_markers) in segments.iter().zip(markers.iter_mut()) {
        sort_native_split_markers(source_markers, policy).ok_or(CurveError::Topology(
            "native split markers could not be sorted".into(),
        ))?;
        for pair in source_markers.windows(2) {
            let start_param = pair[0].param.clone();
            let end_param = pair[1].param.clone();
            match compare_reals(&start_param, &end_param, policy) {
                Some(Ordering::Less) => {
                    match materialize_native_segment_between_markers(
                        segment, &pair[0], &pair[1], policy,
                    )? {
                        NativeSegmentMaterialization::Materialized(fragment) => {
                            arranged.push(ArrangedNativeSegment { segment: fragment });
                        }
                        NativeSegmentMaterialization::SkippedEmpty => {}
                        NativeSegmentMaterialization::Unresolved(reason) => {
                            return Ok(Err(reason));
                        }
                    }
                }
                Some(Ordering::Equal) => {}
                Some(Ordering::Greater) | None => {
                    return Ok(Err(UncertaintyReason::Ordering));
                }
            }
        }
    }

    Ok(Ok(ArrangedNativeSegments { segments: arranged }))
}

#[derive(Clone, Debug, PartialEq)]
struct NativeSegmentIntersectionPoint {
    point: Point2,
    first_param: Real,
    second_param: Real,
}

#[derive(Clone, Debug, PartialEq)]
enum NativeSegmentIntersectionMarkers {
    None,
    Points(Vec<NativeSegmentIntersectionPoint>),
    Overlap,
    Uncertain(UncertaintyReason),
}

fn native_segment_intersection_split_markers(
    first: &Segment2,
    second: &Segment2,
    policy: &CurveContext,
) -> CurveResult<NativeSegmentIntersectionMarkers> {
    match first.intersect_segment(second, policy)? {
        SegmentIntersection::LineLine(LineLineIntersection::None) => {
            Ok(NativeSegmentIntersectionMarkers::None)
        }
        SegmentIntersection::LineLine(LineLineIntersection::Point {
            point,
            a_param,
            b_param,
            ..
        }) => Ok(NativeSegmentIntersectionMarkers::Points(vec![
            NativeSegmentIntersectionPoint {
                point,
                first_param: a_param,
                second_param: b_param,
            },
        ])),
        SegmentIntersection::LineLine(LineLineIntersection::Overlap { .. }) => {
            Ok(NativeSegmentIntersectionMarkers::Overlap)
        }
        SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason }) => {
            Ok(NativeSegmentIntersectionMarkers::Uncertain(reason))
        }
        SegmentIntersection::LineArc { order, result } => {
            native_line_arc_intersection_split_markers(order, result)
        }
        SegmentIntersection::ArcArc(result) => native_arc_arc_intersection_split_markers(result),
    }
}

fn native_line_arc_intersection_split_markers(
    order: LineArcOrder,
    result: LineArcIntersection,
) -> CurveResult<NativeSegmentIntersectionMarkers> {
    let map_point = |hit: crate::LineArcIntersectionPoint| {
        let (first_param, second_param) = match order {
            LineArcOrder::LineThenArc => (hit.line_param, hit.arc_param),
            LineArcOrder::ArcThenLine => (hit.arc_param, hit.line_param),
        };
        NativeSegmentIntersectionPoint {
            point: hit.point,
            first_param,
            second_param,
        }
    };

    Ok(match result {
        LineArcIntersection::None => NativeSegmentIntersectionMarkers::None,
        LineArcIntersection::Point(hit) => {
            NativeSegmentIntersectionMarkers::Points(vec![map_point(hit)])
        }
        LineArcIntersection::TwoPoints { first, second } => {
            NativeSegmentIntersectionMarkers::Points(vec![map_point(first), map_point(second)])
        }
        LineArcIntersection::Uncertain { reason } => {
            NativeSegmentIntersectionMarkers::Uncertain(reason)
        }
    })
}

fn native_arc_arc_intersection_split_markers(
    result: ArcArcIntersection,
) -> CurveResult<NativeSegmentIntersectionMarkers> {
    Ok(match result {
        ArcArcIntersection::None => NativeSegmentIntersectionMarkers::None,
        ArcArcIntersection::Point(hit) => {
            NativeSegmentIntersectionMarkers::Points(vec![NativeSegmentIntersectionPoint {
                point: hit.point,
                first_param: hit.a_param,
                second_param: hit.b_param,
            }])
        }
        ArcArcIntersection::TwoPoints { first, second } => {
            NativeSegmentIntersectionMarkers::Points(vec![
                NativeSegmentIntersectionPoint {
                    point: first.point,
                    first_param: first.a_param,
                    second_param: first.b_param,
                },
                NativeSegmentIntersectionPoint {
                    point: second.point,
                    first_param: second.a_param,
                    second_param: second.b_param,
                },
            ])
        }
        ArcArcIntersection::Overlap { .. } => NativeSegmentIntersectionMarkers::Overlap,
        ArcArcIntersection::Uncertain { reason } => {
            NativeSegmentIntersectionMarkers::Uncertain(reason)
        }
    })
}

fn insert_native_split_marker(
    markers: &mut Vec<NativeSegmentSplitMarker>,
    marker: NativeSegmentSplitMarker,
    policy: &CurveContext,
) -> Option<()> {
    for existing in markers.iter() {
        if compare_reals(&existing.param, &marker.param, policy)? == Ordering::Equal {
            return match crate::classify::is_zero(
                &existing.point.distance_squared(&marker.point),
                policy,
            ) {
                Some(true) => Some(()),
                Some(false) | None => None,
            };
        }
    }
    markers.push(marker);
    Some(())
}

fn sort_native_split_markers(
    markers: &mut [NativeSegmentSplitMarker],
    policy: &CurveContext,
) -> Option<()> {
    let mut failed = false;
    markers.sort_by(|left, right| {
        compare_reals(&left.param, &right.param, policy).unwrap_or_else(|| {
            failed = true;
            Ordering::Equal
        })
    });
    (!failed).then_some(())
}

enum NativeSegmentMaterialization {
    Materialized(Segment2),
    SkippedEmpty,
    Unresolved(UncertaintyReason),
}

fn materialize_native_segment_between_markers(
    source_segment: &Segment2,
    start: &NativeSegmentSplitMarker,
    end: &NativeSegmentSplitMarker,
    policy: &CurveContext,
) -> CurveResult<NativeSegmentMaterialization> {
    match crate::classify::is_zero(&start.point.distance_squared(&end.point), policy) {
        Some(true) => return Ok(NativeSegmentMaterialization::SkippedEmpty),
        Some(false) => {}
        None => {
            return Ok(NativeSegmentMaterialization::Unresolved(
                UncertaintyReason::RealSign,
            ));
        }
    }

    match source_segment {
        Segment2::Line(_) => LineSeg2::try_new(start.point.clone(), end.point.clone())
            .map(Segment2::Line)
            .map(NativeSegmentMaterialization::Materialized),
        Segment2::Arc(arc) => {
            materialize_arc_between_markers(arc, &start.point, &end.point, policy)
        }
    }
}

fn materialize_arc_between_markers(
    source_arc: &CircularArc2,
    start: &Point2,
    end: &Point2,
    policy: &CurveContext,
) -> CurveResult<NativeSegmentMaterialization> {
    match (
        source_arc.contains_point(start, policy),
        source_arc.contains_point(end, policy),
    ) {
        (Classification::Decided(true), Classification::Decided(true)) => {
            Ok(NativeSegmentMaterialization::Materialized(Segment2::Arc(
                CircularArc2::new_unchecked_with_radius(
                    start.clone(),
                    end.clone(),
                    source_arc.center().clone(),
                    source_arc.radius_squared(),
                    source_arc.is_clockwise(),
                    None,
                ),
            )))
        }
        (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {
            Err(CurveError::InvalidCurveRange)
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Ok(NativeSegmentMaterialization::Unresolved(reason))
        }
    }
}

fn assemble_unordered_line_segment_rings(
    segments: &[ArrangedLineSegment],
    policy: &CurveContext,
) -> Result<Vec<Vec<LineSeg2>>, UncertaintyReason> {
    let mut used = vec![false; segments.len()];
    let mut rings = Vec::new();

    while let Some(seed_index) = used.iter().position(|used| !*used) {
        let mut ring = Vec::new();
        let mut current = segments[seed_index].clone();
        used[seed_index] = true;
        let ring_start = current.line.start().clone();
        ring.push(current.line.clone());

        loop {
            match exact_points_match(current.line.end(), &ring_start, policy) {
                Classification::Decided(true) => break,
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Err(reason),
            }

            let next = match unique_next_line_segment(current.line.end(), segments, &used, policy) {
                Classification::Decided(Some(next)) => next,
                Classification::Decided(None) => return Err(UncertaintyReason::Boundary),
                Classification::Uncertain(reason) => return Err(reason),
            };

            used[next.arranged_segment_index] = true;
            current = if next.reversed {
                segments[next.arranged_segment_index].reversed()
            } else {
                segments[next.arranged_segment_index].clone()
            };
            ring.push(current.line.clone());
        }

        if ring.len() < 3 {
            return Err(UncertaintyReason::Boundary);
        }
        canonicalize_line_ring_endpoints(&mut ring);
        rings.push(ring);
    }

    Ok(rings)
}

fn canonicalize_line_ring_endpoints(ring: &mut [LineSeg2]) {
    for index in 1..ring.len() {
        ring[index] =
            LineSeg2::new_unchecked(ring[index - 1].end().clone(), ring[index].end().clone());
    }
    if let Some((first, rest)) = ring.split_first_mut()
        && let Some(last) = rest.last_mut()
    {
        *last = LineSeg2::new_unchecked(last.start().clone(), first.start().clone());
    }
}

#[derive(Clone, Debug, PartialEq)]
struct NextLineSegment {
    arranged_segment_index: usize,
    reversed: bool,
}

fn unique_next_line_segment(
    target: &Point2,
    segments: &[ArrangedLineSegment],
    used: &[bool],
    policy: &CurveContext,
) -> Classification<Option<NextLineSegment>> {
    let mut selected = None;
    for (arranged_segment_index, segment) in segments.iter().enumerate() {
        if used[arranged_segment_index] {
            continue;
        }
        for candidate in [EndpointCandidate::Start, EndpointCandidate::End] {
            let point = match candidate {
                EndpointCandidate::Start => segment.line.start(),
                EndpointCandidate::End => segment.line.end(),
            };
            match exact_points_match(target, point, policy) {
                Classification::Decided(true) => {
                    if selected.is_some() {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                    selected = Some(NextLineSegment {
                        arranged_segment_index,
                        reversed: candidate == EndpointCandidate::End,
                    });
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
    }
    Classification::Decided(selected)
}

fn assemble_unordered_native_segment_rings(
    segments: &[ArrangedNativeSegment],
    policy: &CurveContext,
) -> Result<Vec<Vec<Segment2>>, UncertaintyReason> {
    let mut used = vec![false; segments.len()];
    let mut rings = Vec::new();

    while let Some(seed_index) = used.iter().position(|used| !*used) {
        let mut ring = Vec::new();
        let mut current = segments[seed_index].clone();
        used[seed_index] = true;
        let ring_start = current.segment.start().clone();
        ring.push(current.segment.clone());

        loop {
            match exact_points_match(current.segment.end(), &ring_start, policy) {
                Classification::Decided(true) => break,
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Err(reason),
            }

            let next =
                match unique_next_native_segment(current.segment.end(), segments, &used, policy) {
                    Classification::Decided(Some(next)) => next,
                    Classification::Decided(None) => return Err(UncertaintyReason::Boundary),
                    Classification::Uncertain(reason) => return Err(reason),
                };

            used[next.arranged_segment_index] = true;
            current = if next.reversed {
                segments[next.arranged_segment_index].reversed()
            } else {
                segments[next.arranged_segment_index].clone()
            };
            ring.push(current.segment.clone());
        }

        canonicalize_native_ring_endpoints(&mut ring);
        rings.push(ring);
    }

    Ok(rings)
}

fn canonicalize_native_ring_endpoints(ring: &mut [Segment2]) {
    for index in 1..ring.len() {
        ring[index] = native_segment_with_endpoints(
            &ring[index],
            ring[index - 1].end().clone(),
            ring[index].end().clone(),
        );
    }
    if let Some((first, rest)) = ring.split_first_mut() {
        let ring_start = first.start().clone();
        if let Some(last) = rest.last_mut() {
            *last = native_segment_with_endpoints(last, last.start().clone(), ring_start);
        } else {
            *first = native_segment_with_endpoints(first, ring_start.clone(), ring_start);
        }
    }
}

fn native_segment_with_endpoints(segment: &Segment2, start: Point2, end: Point2) -> Segment2 {
    match segment {
        Segment2::Line(_) => Segment2::Line(LineSeg2::new_unchecked(start, end)),
        Segment2::Arc(arc) => Segment2::Arc(CircularArc2::new_unchecked_with_radius(
            start,
            end,
            arc.center().clone(),
            arc.radius_squared(),
            arc.is_clockwise(),
            None,
        )),
    }
}

fn unique_next_native_segment(
    target: &Point2,
    segments: &[ArrangedNativeSegment],
    used: &[bool],
    policy: &CurveContext,
) -> Classification<Option<NextLineSegment>> {
    let mut selected = None;
    for (arranged_segment_index, segment) in segments.iter().enumerate() {
        if used[arranged_segment_index] {
            continue;
        }
        for candidate in [EndpointCandidate::Start, EndpointCandidate::End] {
            let point = match candidate {
                EndpointCandidate::Start => segment.segment.start(),
                EndpointCandidate::End => segment.segment.end(),
            };
            match exact_points_match(target, point, policy) {
                Classification::Decided(true) => {
                    if selected.is_some() {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                    selected = Some(NextLineSegment {
                        arranged_segment_index,
                        reversed: candidate == EndpointCandidate::End,
                    });
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
    }
    Classification::Decided(selected)
}

fn exact_points_match(
    left: &Point2,
    right: &Point2,
    policy: &CurveContext,
) -> Classification<bool> {
    match crate::classify::is_zero(&left.distance_squared(right), policy) {
        Some(value) => Classification::Decided(value),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}
fn region_segment_kind_counts(region: &LineArcRegion2) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in region
        .material_contours()
        .iter()
        .chain(region.hole_contours().iter())
        .flat_map(|contour| contour.segments())
    {
        add_segment_kind(&mut counts, segment);
    }
    counts
}

fn add_segment_kind(counts: &mut SegmentKindCounts, segment: &Segment2) {
    match segment {
        Segment2::Line(_) => counts.lines += 1,
        Segment2::Arc(_) => counts.arcs += 1,
    }
}

fn blocked_line_segment_region_evidence(
    stage: CurveRegionArrangementStage2,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> RegionLineSegmentRegionBuildEvidence2 {
    RegionLineSegmentRegionBuildEvidence2 {
        stage,
        output_ring_count: None,
        output_boundary_segment_count: None,
        output_boundary_segment_kind_counts: None,
        status,
        blocker: Some(blocker),
    }
}
fn retained_status_for_line_segment_region_blocker(
    blocker: UncertaintyReason,
) -> RetainedTopologyStatus {
    match blocker {
        UncertaintyReason::Boundary | UncertaintyReason::Unsupported => {
            RetainedTopologyStatus::Unsupported
        }
        _ => RetainedTopologyStatus::Unresolved,
    }
}

fn retained_status_for_boundary_contour_blocker(
    reason: UncertaintyReason,
) -> RetainedTopologyStatus {
    match reason {
        UncertaintyReason::Boundary | UncertaintyReason::Unsupported => {
            RetainedTopologyStatus::Unsupported
        }
        _ => RetainedTopologyStatus::Unresolved,
    }
}

fn contour_aabb_overlap_neighbors(
    contour_boxes: &[Option<Aabb2>],
    policy: &CurveContext,
) -> Option<Vec<Vec<usize>>> {
    const MIN_RETAINED_NEIGHBOR_COUNT: usize = 4_096;
    const RETAINED_NEIGHBORS_PER_CONTOUR: usize = 32;

    let retained_neighbor_limit = contour_boxes
        .len()
        .saturating_mul(RETAINED_NEIGHBORS_PER_CONTOUR)
        .max(MIN_RETAINED_NEIGHBOR_COUNT);
    let mut ordered = contour_boxes
        .iter()
        .enumerate()
        .filter_map(|(index, bbox)| bbox.as_ref().map(|bbox| (index, bbox)))
        .collect::<Vec<_>>();
    let mut ordering_failed = false;
    ordered.sort_by(|(_, left), (_, right)| {
        compare_reals(left.min().x(), right.min().x(), policy).unwrap_or_else(|| {
            ordering_failed = true;
            Ordering::Equal
        })
    });
    if ordering_failed {
        return None;
    }

    let mut neighbors = (0..contour_boxes.len())
        .map(|_| Vec::new())
        .collect::<Vec<_>>();
    let mut retained_neighbor_count = 0_usize;
    let mut active = Vec::<(usize, &Aabb2)>::new();
    for (current_index, current_box) in ordered {
        active.retain(|(_, active_box)| {
            compare_reals(active_box.max().x(), current_box.min().x(), policy)
                != Some(Ordering::Less)
        });
        for &(active_index, active_box) in &active {
            if aabbs_decided_disjoint(active_box, current_box, policy) {
                continue;
            }
            retained_neighbor_count += 1;
            if retained_neighbor_count > retained_neighbor_limit {
                return None;
            }
            neighbors[active_index].push(current_index);
            neighbors[current_index].push(active_index);
        }
        active.push((current_index, current_box));
    }

    for undecided_index in contour_boxes
        .iter()
        .enumerate()
        .filter_map(|(index, bbox)| bbox.is_none().then_some(index))
    {
        for other_index in 0..contour_boxes.len() {
            if undecided_index == other_index
                || contour_boxes[other_index].is_none() && other_index < undecided_index
            {
                continue;
            }
            retained_neighbor_count += 1;
            if retained_neighbor_count > retained_neighbor_limit {
                return None;
            }
            neighbors[undecided_index].push(other_index);
            neighbors[other_index].push(undecided_index);
        }
    }

    for contour_neighbors in &mut neighbors {
        contour_neighbors.sort_unstable();
    }
    Some(neighbors)
}

fn aabb_may_contain(outer: &Aabb2, inner: &Aabb2, policy: &CurveContext) -> bool {
    let mut predicates = [
        (
            outer
                .min_x()
                .to_f64_lossy()
                .zip(inner.min_x().to_f64_lossy())
                .map_or(f64::NEG_INFINITY, |(outer, inner)| outer - inner),
            outer.min_x(),
            inner.min_x(),
            Ordering::Greater,
        ),
        (
            outer
                .min_y()
                .to_f64_lossy()
                .zip(inner.min_y().to_f64_lossy())
                .map_or(f64::NEG_INFINITY, |(outer, inner)| outer - inner),
            outer.min_y(),
            inner.min_y(),
            Ordering::Greater,
        ),
        (
            inner
                .max_x()
                .to_f64_lossy()
                .zip(outer.max_x().to_f64_lossy())
                .map_or(f64::NEG_INFINITY, |(inner, outer)| inner - outer),
            outer.max_x(),
            inner.max_x(),
            Ordering::Less,
        ),
        (
            inner
                .max_y()
                .to_f64_lossy()
                .zip(outer.max_y().to_f64_lossy())
                .map_or(f64::NEG_INFINITY, |(inner, outer)| inner - outer),
            outer.max_y(),
            inner.max_y(),
            Ordering::Less,
        ),
    ];
    predicates.sort_by(|left, right| right.0.total_cmp(&left.0));
    predicates
        .into_iter()
        .all(|(_, outer, inner, violation)| compare_reals(outer, inner, policy) != Some(violation))
}

fn contour_nesting_depths(
    contours: &[Contour2],
    policy: &CurveContext,
) -> CurveResult<BoundaryContourNestingOutcome> {
    // Region construction compares every contour pair, then classifies a
    // boundary sample against every other contour. Retain the broad-phase
    // certificates across both passes instead of rebuilding them for every
    // pair and point query. Segment boxes stay lazy because disjoint contour
    // boxes settle the common case without them.
    let contour_boxes = contours
        .iter()
        .map(|contour| decided_contour_aabb(contour, policy))
        .collect::<Vec<_>>();
    let segment_boxes = (0..contours.len())
        .map(|_| OnceLock::<Vec<Option<Aabb2>>>::new())
        .collect::<Vec<_>>();
    let prepared_contours = (0..contours.len())
        .map(|_| OnceLock::<crate::prepared::ContourQuery2<'_>>::new())
        .collect::<Vec<_>>();
    let aabb_overlap_neighbors = contour_aabb_overlap_neighbors(&contour_boxes, policy);

    for (left_index, left) in contours.iter().enumerate() {
        let mut neighbor_position = aabb_overlap_neighbors.as_ref().map_or(0, |neighbors| {
            neighbors[left_index].partition_point(|&index| index <= left_index)
        });
        for (right_offset, right) in contours[left_index + 1..].iter().enumerate() {
            let right_index = left_index + 1 + right_offset;
            if let Some(neighbors) = &aabb_overlap_neighbors {
                if neighbors[left_index].get(neighbor_position).copied() != Some(right_index) {
                    continue;
                }
                neighbor_position += 1;
            }
            if let (Some(left_box), Some(right_box)) = (
                contour_boxes[left_index].as_ref(),
                contour_boxes[right_index].as_ref(),
            ) && aabbs_decided_disjoint(left_box, right_box, policy)
            {
                continue;
            }
            let intersections = if let (Some(left_boxes), Some(right_boxes)) = (
                left.exact_dyadic_line_aabbs(policy),
                right.exact_dyadic_line_aabbs(policy),
            ) {
                crate::events::intersect_contours_with_exact_dyadic_line_aabbs(
                    left,
                    right,
                    &left_boxes,
                    &right_boxes,
                    policy,
                )?
            } else {
                let left_segment_boxes = segment_boxes[left_index].get_or_init(|| {
                    left.segments()
                        .iter()
                        .map(|segment| decided_segment_aabb(segment, policy))
                        .collect()
                });
                let right_segment_boxes = segment_boxes[right_index].get_or_init(|| {
                    right
                        .segments()
                        .iter()
                        .map(|segment| decided_segment_aabb(segment, policy))
                        .collect()
                });
                crate::events::intersect_contours_with_cached_aabbs(
                    left,
                    right,
                    contour_boxes[left_index].as_ref(),
                    contour_boxes[right_index].as_ref(),
                    left_segment_boxes,
                    right_segment_boxes,
                    None,
                    policy,
                )?
            };
            if let Some(reason) = contour_intersection_blocker(&intersections) {
                return Ok(BoundaryContourNestingOutcome::Blocked(reason));
            }
        }
    }

    let mut depths = Vec::with_capacity(contours.len());

    for (candidate_index, candidate) in contours.iter().enumerate() {
        // A point on the candidate boundary is sufficient for nesting against
        // every *other* non-touching contour. This reduces role assignment to
        // repeated point-in-polygon classification. If that sample lies on
        // another contour boundary, return uncertainty instead of inventing a
        // role.
        let first_sample = candidate
            .segments()
            .first()
            .ok_or(CurveError::EmptyCurveString)?
            .start()
            .clone();
        let fractions = [
            (Real::one() / Real::from(2_i8))?,
            (Real::one() / Real::from(3_i8))?,
            (Real::from(2_i8) / Real::from(3_i8))?,
        ];
        // The exact validation pass above, or the trusted Boolean assembly
        // supplying this internal path, proves that every point on this
        // candidate boundary is off every other contour boundary. Its first
        // endpoint is therefore a sufficient nesting sample. Retain interior
        // samples as exact fallbacks for an undecided containment predicate,
        // but do not eagerly interpolate them when that endpoint decides.
        let fallback_samples = candidate.segments().iter().flat_map(|segment| {
            fractions
                .iter()
                .map(move |fraction| segment.point_at(fraction, policy))
        });
        let samples =
            std::iter::once(Ok(Classification::Decided(first_sample))).chain(fallback_samples);

        let mut last_blocker = None;
        let mut decided_entry = None;
        'sample: for sample in samples {
            let Classification::Decided(sample) = sample? else {
                continue;
            };
            let mut depth = 0_usize;
            let mut neighbor_position = 0;
            for (container_index, container) in contours.iter().enumerate() {
                if candidate_index == container_index {
                    continue;
                }

                if let Some(neighbors) = &aabb_overlap_neighbors {
                    if neighbors[candidate_index].get(neighbor_position).copied()
                        != Some(container_index)
                    {
                        continue;
                    }
                    neighbor_position += 1;
                }
                if let (Some(container_box), Some(candidate_box)) = (
                    contour_boxes[container_index].as_ref(),
                    contour_boxes[candidate_index].as_ref(),
                ) && !aabb_may_contain(container_box, candidate_box, policy)
                {
                    continue;
                }
                if contour_boxes[container_index]
                    .as_ref()
                    .is_some_and(|bbox| aabb_decided_misses_point(bbox, &sample, policy))
                {
                    continue;
                }
                let prepared = prepared_contours[container_index].get_or_init(|| {
                    crate::prepared::ContourQuery2::from_contour(container, policy)
                });
                match prepared.classify_point_assuming_off_boundary(&sample, policy) {
                    Classification::Decided(ContourPointLocation::Inside) => {
                        depth += 1;
                    }
                    Classification::Decided(ContourPointLocation::Outside) => {}
                    Classification::Decided(ContourPointLocation::Boundary) => {
                        return Ok(BoundaryContourNestingOutcome::Blocked(
                            crate::UncertaintyReason::Boundary,
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        last_blocker = Some(reason);
                        continue 'sample;
                    }
                }
            }
            decided_entry = Some(depth);
            break;
        }

        let Some(depth) = decided_entry else {
            return Ok(BoundaryContourNestingOutcome::Blocked(
                last_blocker.unwrap_or(crate::UncertaintyReason::Unsupported),
            ));
        };
        depths.push(depth);
    }

    Ok(BoundaryContourNestingOutcome::Decided(
        BoundaryContourNestingDepths { depths },
    ))
}

fn contour_intersection_blocker(
    intersections: &crate::ContourIntersectionSet,
) -> Option<UncertaintyReason> {
    let mut uncertainty = None;
    for event in intersections.events() {
        match event {
            crate::ContourIntersection::Point(_) | crate::ContourIntersection::Overlap(_) => {
                return Some(UncertaintyReason::Boundary);
            }
            crate::ContourIntersection::Uncertain(event) => {
                uncertainty.get_or_insert(event.reason);
            }
        }
    }
    uncertainty
}

#[cfg(test)]
mod tests {
    use super::contour_intersection_blocker;
    use crate::{
        ContourIntersection, ContourIntersectionSet, ContourPointIntersection,
        ContourUncertainIntersection, IntersectionKind, Point2, SegmentKind, UncertaintyReason,
    };
    use hyperreal::Real;

    #[test]
    fn contour_nesting_preserves_uncertain_intersection_reason() {
        let intersections = ContourIntersectionSet::new(vec![ContourIntersection::Uncertain(
            ContourUncertainIntersection {
                a_segment_index: 2,
                b_segment_index: 4,
                a_segment_kind: SegmentKind::Arc,
                b_segment_kind: SegmentKind::Line,
                reason: UncertaintyReason::RealSign,
            },
        )])
        .unwrap();

        assert_eq!(
            contour_intersection_blocker(&intersections),
            Some(UncertaintyReason::RealSign)
        );
    }

    #[test]
    fn contour_nesting_prefers_decided_contact_over_uncertainty() {
        let intersections = ContourIntersectionSet::new(vec![
            ContourIntersection::Uncertain(ContourUncertainIntersection {
                a_segment_index: 0,
                b_segment_index: 0,
                a_segment_kind: SegmentKind::Line,
                b_segment_kind: SegmentKind::Line,
                reason: UncertaintyReason::Ordering,
            }),
            ContourIntersection::Point(ContourPointIntersection {
                a_segment_index: 1,
                b_segment_index: 1,
                a_segment_kind: SegmentKind::Line,
                b_segment_kind: SegmentKind::Line,
                point: Point2::new(Real::zero(), Real::zero()),
                a_param: Real::zero(),
                b_param: Real::zero(),
                kind: IntersectionKind::Endpoint,
            }),
        ])
        .unwrap();

        assert_eq!(
            contour_intersection_blocker(&intersections),
            Some(UncertaintyReason::Boundary)
        );
    }
}
