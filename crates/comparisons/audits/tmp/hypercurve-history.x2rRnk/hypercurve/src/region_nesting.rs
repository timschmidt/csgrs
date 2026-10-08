//! Contour nesting and material/hole role assignment.
//!
//! This module assembles unordered native segments into closed contours. Final
//! material/hole roles are delegated to [`crate::CurveRegion2`]'s all-family
//! curve-pair validation and curved-loop nesting authority.
use std::cmp::Ordering;

use hyperreal::Real;

use crate::bbox::{Aabb2, aabbs_decided_disjoint};
use crate::bezier_region::CurveRegionArrangementStage2;
use crate::classify::compare_reals;
use crate::region::LineArcRegion2;
use crate::{
    ArcArcIntersection, CircularArc2, Classification, Contour2, CurveContext, CurveError,
    CurveResult, FillRule, LineArcIntersection, LineArcOrder, LineLineIntersection, LineSeg2,
    Point2, RetainedTopologyStatus, Segment2, SegmentIntersection, SegmentKindCounts,
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
    let nesting = match crate::CurveRegion2::native_boundary_contour_nesting_evidence_raw(
        &contours, policy,
    ) {
        Ok(Classification::Decided(nesting)) => nesting,
        Ok(Classification::Uncertain(reason)) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                evidence: blocked_line_segment_region_evidence(
                    CurveRegionArrangementStage2::RegionRoleAssignment,
                    retained_status_for_boundary_contour_blocker(reason),
                    reason,
                ),
            });
        }
        Err(crate::ExactCurveError::Invalid { cause, .. }) => return Err(cause),
        Err(crate::ExactCurveError::Blocked(blocker)) => {
            let reason = blocker.reason();
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                evidence: blocked_line_segment_region_evidence(
                    CurveRegionArrangementStage2::RegionRoleAssignment,
                    retained_status_for_boundary_contour_blocker(reason),
                    reason,
                ),
            });
        }
    };
    let region = assign_boundary_contour_roles(contours, nesting.roles());
    let output_ring_count = Some(region.material_contours().len() + region.hole_contours().len());
    let output_boundary_segment_count = Some(
        region
            .material_contours()
            .iter()
            .chain(region.hole_contours())
            .map(|contour| contour.segments().len())
            .sum(),
    );
    let output_boundary_segment_kind_counts = Some(region_segment_kind_counts(&region));
    Ok(RegionLineSegmentRegionBuildResult2 {
        region: Some(region),
        evidence: RegionLineSegmentRegionBuildEvidence2 {
            stage: CurveRegionArrangementStage2::RegionRoleAssignment,
            output_ring_count,
            output_boundary_segment_count,
            output_boundary_segment_kind_counts,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
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
}

fn assign_boundary_contour_roles(
    contours: Vec<Contour2>,
    roles: &[crate::CurveRegionLoopRole],
) -> LineArcRegion2 {
    let mut material_contours = Vec::new();
    let mut hole_contours = Vec::new();
    for (contour, role) in contours.into_iter().zip(roles) {
        match role {
            crate::CurveRegionLoopRole::Material => material_contours.push(contour),
            crate::CurveRegionLoopRole::Hole => hole_contours.push(contour),
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
