//! Contour nesting and material/hole role assignment.
//!
//! This module turns already-closed boundary contours into the signed contour
//! bins used by [`crate::Region2`]. It assumes intersections and overlaps have
//! already been resolved by earlier topology stages.

use crate::{
    Classification, Contour2, ContourPointLocation, CurveError, CurvePolicy, CurveResult, FillRule,
    LineSeg2, Point2, Region2, RetainedTopologyStatus, Segment2, UncertaintyReason,
};

/// Material/hole role assigned to one closed boundary contour.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionBoundaryContourRole2 {
    /// The contour contributes filled material.
    Material,
    /// The contour contributes a subtractive hole.
    Hole,
}

/// Role assignment for one source boundary contour.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBoundaryContourRoleReport2 {
    source_contour_index: usize,
    source_segment_count: usize,
    source_fill_rule: FillRule,
    nesting_sample_point: Point2,
    containing_contour_indices: Vec<usize>,
    nesting_depth: usize,
    role: RegionBoundaryContourRole2,
    output_role_index: usize,
    status: RetainedTopologyStatus,
}

/// Report for building a region from already-closed boundary contours.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBoundaryContourBuildReport2 {
    stage: RegionBoundaryContourBuildStage2,
    source_contour_count: usize,
    source_segment_count: usize,
    validation_candidate_pair_count: usize,
    validation_tested_pair_count: usize,
    validation_intersection_event_count: usize,
    nesting_classification_count: usize,
    blocker_first_contour_index: Option<usize>,
    blocker_second_contour_index: Option<usize>,
    output_contour_count: Option<usize>,
    output_segment_count: Option<usize>,
    material_contour_count: Option<usize>,
    hole_contour_count: Option<usize>,
    material_segment_count: Option<usize>,
    hole_segment_count: Option<usize>,
    role_reports: Vec<RegionBoundaryContourRoleReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by boundary-contour region construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionBoundaryContourBuildStage2 {
    /// Contour intersections and containment nesting were being validated.
    NestingValidation,
    /// Material and hole role bins were assigned and materialized.
    RoleAssignment,
}

/// Result of report-bearing boundary contour region construction.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBoundaryContourBuildResult2 {
    region: Option<Region2>,
    report: RegionBoundaryContourBuildReport2,
}

/// Source line-segment provenance for one assembled boundary ring segment.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentRingSourceReport2 {
    source_segment_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
    reversed: bool,
    output_start_point: Point2,
    output_end_point: Point2,
    status: RetainedTopologyStatus,
}

/// Report for constructing a region from unordered exact line segments.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentRegionBuildReport2 {
    stage: RegionLineSegmentRegionBuildStage2,
    source_segment_count: usize,
    attempted_endpoint_connection_count: usize,
    exact_endpoint_connection_count: usize,
    disconnected_endpoint_connection_count: usize,
    unresolved_endpoint_connection_count: usize,
    reversed_source_segment_count: usize,
    output_ring_count: Option<usize>,
    output_boundary_segment_count: Option<usize>,
    source_reports: Vec<RegionLineSegmentRingSourceReport2>,
    boundary_build_report: Option<RegionBoundaryContourBuildReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached while assembling unordered line segments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionLineSegmentRegionBuildStage2 {
    /// The unordered endpoint graph was being assembled into closed rings.
    RingAssembly,
    /// Assembled line rings were being replayed as checked contours.
    ContourMaterialization,
    /// Checked contours were being assigned material/hole roles.
    RegionRoleAssignment,
}

/// Result of report-bearing region construction from unordered exact line segments.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentRegionBuildResult2 {
    region: Option<Region2>,
    report: RegionLineSegmentRegionBuildReport2,
}

#[derive(Clone, Debug, PartialEq)]
struct BoundaryContourNestingDepths {
    entries: Vec<BoundaryContourNestingEntry>,
}

#[derive(Clone, Debug, PartialEq)]
struct BoundaryContourNestingEntry {
    sample_point: Point2,
    containing_contour_indices: Vec<usize>,
}

#[derive(Clone, Debug, PartialEq)]
struct BoundaryContourNestingBlocker {
    reason: UncertaintyReason,
    first_contour_index: usize,
    second_contour_index: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct BoundaryContourValidationCounts {
    candidate_pair_count: usize,
    tested_pair_count: usize,
    intersection_event_count: usize,
    nesting_classification_count: usize,
}

#[derive(Clone, Debug, PartialEq)]
enum BoundaryContourNestingOutcome {
    Decided {
        nesting: BoundaryContourNestingDepths,
        counts: BoundaryContourValidationCounts,
    },
    Blocked {
        blocker: BoundaryContourNestingBlocker,
        counts: BoundaryContourValidationCounts,
    },
}

impl Region2 {
    /// Builds a region from unordered exact line segments that form closed rings.
    ///
    /// This is a narrow first utility for "make region from lines" workflows:
    /// it accepts already-authored finite line segments, chooses connections
    /// only from exact endpoint equality, reorients source segments as needed,
    /// materializes checked contours, and then delegates material/hole role
    /// assignment to [`Region2::from_boundary_contours_with_report`]. It does
    /// not snap endpoints or split interior crossings; disconnected, ambiguous,
    /// unresolved, or branching endpoint graphs are returned as explicit
    /// blockers.
    pub fn from_unordered_line_segments(
        segments: Vec<LineSeg2>,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let built = Self::from_unordered_line_segments_with_report(segments, fill_rule, policy)?;
        let blocker = built
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(region) = built.into_region() {
            Ok(Classification::Decided(region))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    /// Builds a region from unordered exact line segments and retains assembly evidence.
    pub fn from_unordered_line_segments_with_report(
        segments: Vec<LineSeg2>,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
        if segments.is_empty() {
            return Err(CurveError::EmptyCurveString);
        }

        let assembled = match assemble_unordered_line_segment_rings(&segments, policy)? {
            Ok(assembled) => assembled,
            Err((report, blocker)) => {
                return Ok(RegionLineSegmentRegionBuildResult2 {
                    region: None,
                    report: blocked_line_segment_region_report(
                        segments.len(),
                        report,
                        RegionLineSegmentRegionBuildStage2::RingAssembly,
                        retained_status_for_line_segment_region_blocker(blocker),
                        blocker,
                    ),
                });
            }
        };

        let mut contours = Vec::with_capacity(assembled.rings.len());
        for ring in assembled.rings {
            let contour = Contour2::try_new_with_fill_rule(
                ring.into_iter().map(Segment2::Line).collect(),
                fill_rule,
            )?;
            contours.push(contour);
        }

        let built = Region2::from_boundary_contours_with_report(contours, policy)?;
        let status = built.report().status();
        let blocker = built.report().blocker();
        let boundary_build_report = built.report().clone();
        let output_ring_count = boundary_build_report.output_contour_count();
        let output_boundary_segment_count = boundary_build_report.output_segment_count();
        Ok(RegionLineSegmentRegionBuildResult2 {
            region: built.into_region(),
            report: RegionLineSegmentRegionBuildReport2 {
                stage: RegionLineSegmentRegionBuildStage2::RegionRoleAssignment,
                source_segment_count: segments.len(),
                attempted_endpoint_connection_count: assembled
                    .counts
                    .attempted_endpoint_connection_count,
                exact_endpoint_connection_count: assembled.counts.exact_endpoint_connection_count,
                disconnected_endpoint_connection_count: assembled
                    .counts
                    .disconnected_endpoint_connection_count,
                unresolved_endpoint_connection_count: assembled
                    .counts
                    .unresolved_endpoint_connection_count,
                reversed_source_segment_count: assembled.reversed_source_segment_count,
                output_ring_count,
                output_boundary_segment_count,
                source_reports: assembled.source_reports,
                boundary_build_report: Some(boundary_build_report),
                status,
                blocker,
            },
        })
    }

    /// Builds a region by nesting closed boundary contours into material/hole bins.
    ///
    /// Contours at even containment depth become material. Contours at odd
    /// depth become holes. This matches the even-odd nesting interpretation
    /// commonly used after boolean traversal has produced disjoint closed
    /// output loops.
    pub fn from_boundary_contours(
        contours: Vec<Contour2>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let built = Self::from_boundary_contours_with_report(contours, policy)?;
        let blocker = built
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(region) = built.into_region() {
            Ok(Classification::Decided(region))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    /// Builds a region by nesting closed boundary contours and retaining role evidence.
    ///
    /// This is the report-bearing counterpart to
    /// [`Region2::from_boundary_contours`]. Contours at even containment depth
    /// become material and odd-depth contours become holes. If intersections,
    /// touches, or undecided containment predicates prevent role assignment, no
    /// region is materialized and the report carries the blocker.
    pub fn from_boundary_contours_with_report(
        contours: Vec<Contour2>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionBoundaryContourBuildResult2> {
        let source_contour_count = contours.len();
        let source_segment_count = contours
            .iter()
            .map(|contour| contour.segments().len())
            .sum();
        let (nesting, counts) = match contour_nesting_depths(&contours, policy)? {
            BoundaryContourNestingOutcome::Decided { nesting, counts } => (nesting, counts),
            BoundaryContourNestingOutcome::Blocked { blocker, counts } => {
                return Ok(blocked_boundary_contour_region_result(
                    source_contour_count,
                    source_segment_count,
                    counts,
                    Some((blocker.first_contour_index, blocker.second_contour_index)),
                    retained_status_for_boundary_contour_blocker(blocker.reason),
                    blocker.reason,
                ));
            }
        };
        let mut material_contours = Vec::new();
        let mut hole_contours = Vec::new();
        let mut role_reports = Vec::with_capacity(source_contour_count);

        for (source_contour_index, (contour, entry)) in
            contours.into_iter().zip(nesting.entries.iter()).enumerate()
        {
            let source_segment_count = contour.segments().len();
            let source_fill_rule = contour.fill_rule();
            let depth = entry.containing_contour_indices.len();
            if depth % 2 == 0 {
                let output_role_index = material_contours.len();
                material_contours.push(contour);
                role_reports.push(RegionBoundaryContourRoleReport2 {
                    source_contour_index,
                    source_segment_count,
                    source_fill_rule,
                    nesting_sample_point: entry.sample_point.clone(),
                    containing_contour_indices: entry.containing_contour_indices.clone(),
                    nesting_depth: depth,
                    role: RegionBoundaryContourRole2::Material,
                    output_role_index,
                    status: RetainedTopologyStatus::NativeExact,
                });
            } else {
                let output_role_index = hole_contours.len();
                hole_contours.push(contour);
                role_reports.push(RegionBoundaryContourRoleReport2 {
                    source_contour_index,
                    source_segment_count,
                    source_fill_rule,
                    nesting_sample_point: entry.sample_point.clone(),
                    containing_contour_indices: entry.containing_contour_indices.clone(),
                    nesting_depth: depth,
                    role: RegionBoundaryContourRole2::Hole,
                    output_role_index,
                    status: RetainedTopologyStatus::NativeExact,
                });
            }
        }

        let material_contour_count = material_contours.len();
        let hole_contour_count = hole_contours.len();
        let output_contour_count = material_contour_count + hole_contour_count;
        let material_segment_count = role_reports
            .iter()
            .filter(|report| report.role == RegionBoundaryContourRole2::Material)
            .map(|report| report.source_segment_count)
            .sum();
        let hole_segment_count = role_reports
            .iter()
            .filter(|report| report.role == RegionBoundaryContourRole2::Hole)
            .map(|report| report.source_segment_count)
            .sum();
        let output_segment_count = material_segment_count + hole_segment_count;
        Ok(RegionBoundaryContourBuildResult2 {
            region: Some(Region2::new(material_contours, hole_contours)),
            report: RegionBoundaryContourBuildReport2 {
                stage: RegionBoundaryContourBuildStage2::RoleAssignment,
                source_contour_count,
                source_segment_count,
                validation_candidate_pair_count: counts.candidate_pair_count,
                validation_tested_pair_count: counts.tested_pair_count,
                validation_intersection_event_count: counts.intersection_event_count,
                nesting_classification_count: counts.nesting_classification_count,
                blocker_first_contour_index: None,
                blocker_second_contour_index: None,
                output_contour_count: Some(output_contour_count),
                output_segment_count: Some(output_segment_count),
                material_contour_count: Some(material_contour_count),
                hole_contour_count: Some(hole_contour_count),
                material_segment_count: Some(material_segment_count),
                hole_segment_count: Some(hole_segment_count),
                role_reports,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }
}

impl RegionBoundaryContourRoleReport2 {
    /// Returns the source contour index assigned by this report.
    pub const fn source_contour_index(&self) -> usize {
        self.source_contour_index
    }

    /// Returns the source contour segment count captured before role binning.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the source contour fill rule captured before role binning.
    pub const fn source_fill_rule(&self) -> FillRule {
        self.source_fill_rule
    }

    /// Returns the exact source point used for containment classification.
    pub const fn nesting_sample_point(&self) -> &Point2 {
        &self.nesting_sample_point
    }

    /// Returns source contour indices that exactly contained the sample point.
    pub fn containing_contour_indices(&self) -> &[usize] {
        &self.containing_contour_indices
    }

    /// Returns exact containment depth used for material/hole parity.
    pub const fn nesting_depth(&self) -> usize {
        self.nesting_depth
    }

    /// Returns the assigned material/hole role.
    pub const fn role(&self) -> RegionBoundaryContourRole2 {
        self.role
    }

    /// Returns this contour's index inside its output role bin.
    pub const fn output_role_index(&self) -> usize {
        self.output_role_index
    }

    /// Returns retained topology status for this role assignment.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl RegionBoundaryContourBuildReport2 {
    /// Returns the furthest exact region-construction stage reached.
    pub const fn stage(&self) -> RegionBoundaryContourBuildStage2 {
        self.stage
    }

    /// Returns the number of source boundary contours considered.
    pub const fn source_contour_count(&self) -> usize {
        self.source_contour_count
    }

    /// Returns the total number of source contour segments considered.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the number of contour pairs scheduled for intersection validation.
    pub const fn validation_candidate_pair_count(&self) -> usize {
        self.validation_candidate_pair_count
    }

    /// Returns the number of contour pairs tested before success or a blocker.
    pub const fn validation_tested_pair_count(&self) -> usize {
        self.validation_tested_pair_count
    }

    /// Returns exact contour-intersection events found during nesting validation.
    pub const fn validation_intersection_event_count(&self) -> usize {
        self.validation_intersection_event_count
    }

    /// Returns point-containment classifications used to assign nesting roles.
    pub const fn nesting_classification_count(&self) -> usize {
        self.nesting_classification_count
    }

    /// Returns the first source contour index involved in a blocking relation.
    pub const fn blocker_first_contour_index(&self) -> Option<usize> {
        self.blocker_first_contour_index
    }

    /// Returns the second source contour index involved in a blocking relation.
    pub const fn blocker_second_contour_index(&self) -> Option<usize> {
        self.blocker_second_contour_index
    }

    /// Returns total output contour count when role assignment materialized.
    pub const fn output_contour_count(&self) -> Option<usize> {
        self.output_contour_count
    }

    /// Returns total output boundary segment count when role assignment materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns material contour count when role assignment materialized.
    pub const fn material_contour_count(&self) -> Option<usize> {
        self.material_contour_count
    }

    /// Returns hole contour count when role assignment materialized.
    pub const fn hole_contour_count(&self) -> Option<usize> {
        self.hole_contour_count
    }

    /// Returns material boundary segment count when role assignment materialized.
    pub const fn material_segment_count(&self) -> Option<usize> {
        self.material_segment_count
    }

    /// Returns hole boundary segment count when role assignment materialized.
    pub const fn hole_segment_count(&self) -> Option<usize> {
        self.hole_segment_count
    }

    /// Returns per-contour exact role reports.
    pub fn role_reports(&self) -> &[RegionBoundaryContourRoleReport2] {
        &self.role_reports
    }

    /// Returns region construction status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized construction attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl RegionBoundaryContourBuildResult2 {
    /// Returns the materialized region, if role assignment succeeded.
    pub const fn region(&self) -> Option<&Region2> {
        self.region.as_ref()
    }

    /// Consumes this result and returns the materialized region, if any.
    pub fn into_region(self) -> Option<Region2> {
        self.region
    }

    /// Returns the retained region-construction report.
    pub const fn report(&self) -> &RegionBoundaryContourBuildReport2 {
        &self.report
    }
}

impl RegionLineSegmentRingSourceReport2 {
    /// Returns the source line segment index used by this output segment.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the output ring index.
    pub const fn output_ring_index(&self) -> usize {
        self.output_ring_index
    }

    /// Returns the output segment index inside the ring.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns whether the source line segment was reversed for ring traversal.
    pub const fn reversed(&self) -> bool {
        self.reversed
    }

    /// Returns the emitted segment start point.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the emitted segment end point.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }

    /// Returns retained topology status for this source-to-ring mapping.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl RegionLineSegmentRegionBuildReport2 {
    /// Returns the furthest exact line-region construction stage reached.
    pub const fn stage(&self) -> RegionLineSegmentRegionBuildStage2 {
        self.stage
    }

    /// Returns the number of source line segments considered.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns endpoint pair comparisons attempted during ring assembly.
    pub const fn attempted_endpoint_connection_count(&self) -> usize {
        self.attempted_endpoint_connection_count
    }

    /// Returns endpoint pair comparisons certified as equal.
    pub const fn exact_endpoint_connection_count(&self) -> usize {
        self.exact_endpoint_connection_count
    }

    /// Returns endpoint pair comparisons certified as disconnected.
    pub const fn disconnected_endpoint_connection_count(&self) -> usize {
        self.disconnected_endpoint_connection_count
    }

    /// Returns endpoint pair comparisons whose equality could not be certified.
    pub const fn unresolved_endpoint_connection_count(&self) -> usize {
        self.unresolved_endpoint_connection_count
    }

    /// Returns source segments reversed while materializing ring traversal.
    pub const fn reversed_source_segment_count(&self) -> usize {
        self.reversed_source_segment_count
    }

    /// Returns output ring count when available.
    pub const fn output_ring_count(&self) -> Option<usize> {
        self.output_ring_count
    }

    /// Returns output boundary segment count when available.
    pub const fn output_boundary_segment_count(&self) -> Option<usize> {
        self.output_boundary_segment_count
    }

    /// Returns per-output segment source provenance.
    pub fn source_reports(&self) -> &[RegionLineSegmentRingSourceReport2] {
        &self.source_reports
    }

    /// Returns delegated boundary-contour role assignment evidence, when reached.
    pub const fn boundary_build_report(&self) -> Option<&RegionBoundaryContourBuildReport2> {
        self.boundary_build_report.as_ref()
    }

    /// Returns line-region construction status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized construction attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl RegionLineSegmentRegionBuildResult2 {
    /// Returns the materialized region, if construction succeeded.
    pub const fn region(&self) -> Option<&Region2> {
        self.region.as_ref()
    }

    /// Consumes this result and returns the materialized region, if any.
    pub fn into_region(self) -> Option<Region2> {
        self.region
    }

    /// Returns the retained line-region construction report.
    pub const fn report(&self) -> &RegionLineSegmentRegionBuildReport2 {
        &self.report
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
struct LineSegmentRingAssemblyCounts {
    attempted_endpoint_connection_count: usize,
    exact_endpoint_connection_count: usize,
    disconnected_endpoint_connection_count: usize,
    unresolved_endpoint_connection_count: usize,
}

#[derive(Clone, Debug, PartialEq)]
struct LineSegmentRingAssemblyReportParts {
    counts: LineSegmentRingAssemblyCounts,
    reversed_source_segment_count: usize,
    source_reports: Vec<RegionLineSegmentRingSourceReport2>,
}

#[derive(Clone, Debug, PartialEq)]
struct LineSegmentRingAssembly {
    rings: Vec<Vec<LineSeg2>>,
    counts: LineSegmentRingAssemblyCounts,
    reversed_source_segment_count: usize,
    source_reports: Vec<RegionLineSegmentRingSourceReport2>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum EndpointCandidate {
    Start,
    End,
}

fn assemble_unordered_line_segment_rings(
    segments: &[LineSeg2],
    policy: &CurvePolicy,
) -> CurveResult<
    Result<LineSegmentRingAssembly, (LineSegmentRingAssemblyReportParts, UncertaintyReason)>,
> {
    let mut used = vec![false; segments.len()];
    let mut rings = Vec::new();
    let mut counts = LineSegmentRingAssemblyCounts::default();
    let mut reversed_source_segment_count = 0_usize;
    let mut source_reports = Vec::with_capacity(segments.len());

    while let Some(seed_index) = used.iter().position(|used| !*used) {
        let output_ring_index = rings.len();
        let mut ring = Vec::new();
        let mut current = segments[seed_index].clone();
        used[seed_index] = true;
        append_line_segment_ring_source_report(
            &mut source_reports,
            seed_index,
            output_ring_index,
            ring.len(),
            false,
            &current,
        );
        let ring_start = current.start().clone();
        ring.push(current.clone());

        loop {
            match exact_points_match(current.end(), &ring_start, policy, &mut counts) {
                Classification::Decided(true) => break,
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Err((
                        LineSegmentRingAssemblyReportParts {
                            counts,
                            reversed_source_segment_count,
                            source_reports,
                        },
                        reason,
                    )));
                }
            }

            let next =
                match unique_next_line_segment(current.end(), segments, &used, policy, &mut counts)
                {
                    Classification::Decided(Some(next)) => next,
                    Classification::Decided(None) => {
                        return Ok(Err((
                            LineSegmentRingAssemblyReportParts {
                                counts,
                                reversed_source_segment_count,
                                source_reports,
                            },
                            UncertaintyReason::Boundary,
                        )));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Err((
                            LineSegmentRingAssemblyReportParts {
                                counts,
                                reversed_source_segment_count,
                                source_reports,
                            },
                            reason,
                        )));
                    }
                };

            used[next.source_segment_index] = true;
            if next.reversed {
                reversed_source_segment_count += 1;
            }
            current = if next.reversed {
                segments[next.source_segment_index].reversed()
            } else {
                segments[next.source_segment_index].clone()
            };
            append_line_segment_ring_source_report(
                &mut source_reports,
                next.source_segment_index,
                output_ring_index,
                ring.len(),
                next.reversed,
                &current,
            );
            ring.push(current.clone());
        }

        if ring.len() < 3 {
            return Ok(Err((
                LineSegmentRingAssemblyReportParts {
                    counts,
                    reversed_source_segment_count,
                    source_reports,
                },
                UncertaintyReason::Boundary,
            )));
        }
        rings.push(ring);
    }

    Ok(Ok(LineSegmentRingAssembly {
        rings,
        counts,
        reversed_source_segment_count,
        source_reports,
    }))
}

#[derive(Clone, Debug, PartialEq)]
struct NextLineSegment {
    source_segment_index: usize,
    reversed: bool,
}

fn unique_next_line_segment(
    target: &Point2,
    segments: &[LineSeg2],
    used: &[bool],
    policy: &CurvePolicy,
    counts: &mut LineSegmentRingAssemblyCounts,
) -> Classification<Option<NextLineSegment>> {
    let mut selected = None;
    for (source_segment_index, segment) in segments.iter().enumerate() {
        if used[source_segment_index] {
            continue;
        }
        for candidate in [EndpointCandidate::Start, EndpointCandidate::End] {
            let point = match candidate {
                EndpointCandidate::Start => segment.start(),
                EndpointCandidate::End => segment.end(),
            };
            match exact_points_match(target, point, policy, counts) {
                Classification::Decided(true) => {
                    if selected.is_some() {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                    selected = Some(NextLineSegment {
                        source_segment_index,
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
    policy: &CurvePolicy,
    counts: &mut LineSegmentRingAssemblyCounts,
) -> Classification<bool> {
    counts.attempted_endpoint_connection_count += 1;
    match crate::classify::is_zero(&left.distance_squared(right), policy) {
        Some(true) => {
            counts.exact_endpoint_connection_count += 1;
            Classification::Decided(true)
        }
        Some(false) => {
            counts.disconnected_endpoint_connection_count += 1;
            Classification::Decided(false)
        }
        None => {
            counts.unresolved_endpoint_connection_count += 1;
            Classification::Uncertain(UncertaintyReason::RealSign)
        }
    }
}

fn append_line_segment_ring_source_report(
    source_reports: &mut Vec<RegionLineSegmentRingSourceReport2>,
    source_segment_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
    reversed: bool,
    line: &LineSeg2,
) {
    source_reports.push(RegionLineSegmentRingSourceReport2 {
        source_segment_index,
        output_ring_index,
        output_segment_index,
        reversed,
        output_start_point: line.start().clone(),
        output_end_point: line.end().clone(),
        status: RetainedTopologyStatus::NativeExact,
    });
}

fn blocked_line_segment_region_report(
    source_segment_count: usize,
    report: LineSegmentRingAssemblyReportParts,
    stage: RegionLineSegmentRegionBuildStage2,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> RegionLineSegmentRegionBuildReport2 {
    RegionLineSegmentRegionBuildReport2 {
        stage,
        source_segment_count,
        attempted_endpoint_connection_count: report.counts.attempted_endpoint_connection_count,
        exact_endpoint_connection_count: report.counts.exact_endpoint_connection_count,
        disconnected_endpoint_connection_count: report
            .counts
            .disconnected_endpoint_connection_count,
        unresolved_endpoint_connection_count: report.counts.unresolved_endpoint_connection_count,
        reversed_source_segment_count: report.reversed_source_segment_count,
        output_ring_count: None,
        output_boundary_segment_count: None,
        source_reports: report.source_reports,
        boundary_build_report: None,
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

fn blocked_boundary_contour_region_result(
    source_contour_count: usize,
    source_segment_count: usize,
    counts: BoundaryContourValidationCounts,
    blocker_contour_indices: Option<(usize, usize)>,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> RegionBoundaryContourBuildResult2 {
    let (blocker_first_contour_index, blocker_second_contour_index) =
        blocker_contour_indices.map_or((None, None), |(first, second)| (Some(first), Some(second)));
    RegionBoundaryContourBuildResult2 {
        region: None,
        report: RegionBoundaryContourBuildReport2 {
            stage: RegionBoundaryContourBuildStage2::NestingValidation,
            source_contour_count,
            source_segment_count,
            validation_candidate_pair_count: counts.candidate_pair_count,
            validation_tested_pair_count: counts.tested_pair_count,
            validation_intersection_event_count: counts.intersection_event_count,
            nesting_classification_count: counts.nesting_classification_count,
            blocker_first_contour_index,
            blocker_second_contour_index,
            output_contour_count: None,
            output_segment_count: None,
            material_contour_count: None,
            hole_contour_count: None,
            material_segment_count: None,
            hole_segment_count: None,
            role_reports: Vec::new(),
            status,
            blocker: Some(blocker),
        },
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

fn contour_nesting_depths(
    contours: &[Contour2],
    policy: &CurvePolicy,
) -> CurveResult<BoundaryContourNestingOutcome> {
    let candidate_pair_count = contours
        .len()
        .saturating_mul(contours.len().saturating_sub(1))
        / 2;
    let mut counts = BoundaryContourValidationCounts {
        candidate_pair_count,
        tested_pair_count: 0,
        intersection_event_count: 0,
        nesting_classification_count: 0,
    };

    for (left_index, left) in contours.iter().enumerate() {
        for (right_offset, right) in contours[left_index + 1..].iter().enumerate() {
            counts.tested_pair_count += 1;
            let intersections = left.intersect_contour(right, policy)?;
            counts.intersection_event_count += intersections.len();
            if !intersections.is_empty() {
                return Ok(BoundaryContourNestingOutcome::Blocked {
                    blocker: BoundaryContourNestingBlocker {
                        reason: crate::UncertaintyReason::Boundary,
                        first_contour_index: left_index,
                        second_contour_index: left_index + 1 + right_offset,
                    },
                    counts,
                });
            }
        }
    }

    let mut entries = Vec::with_capacity(contours.len());

    for (candidate_index, candidate) in contours.iter().enumerate() {
        // A point on the candidate boundary is sufficient for nesting against
        // every *other* non-touching contour. This reduces role assignment to
        // repeated point-in-polygon classification, the degeneracy-sensitive
        // problem surveyed by K. Hormann and A. Agathos, "The point in polygon
        // problem for arbitrary polygons," Computational Geometry 20(3),
        // 131-144, 2001. If that sample lies on another contour boundary, we
        // return uncertainty instead of inventing a role.
        let sample = candidate
            .segments()
            .first()
            .ok_or(CurveError::EmptyCurveString)?
            .start();
        let mut containing_contour_indices = Vec::new();

        for (container_index, container) in contours.iter().enumerate() {
            if candidate_index == container_index {
                continue;
            }

            counts.nesting_classification_count += 1;
            match container.classify_point(sample, policy) {
                Classification::Decided(ContourPointLocation::Inside) => {
                    containing_contour_indices.push(container_index);
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(BoundaryContourNestingOutcome::Blocked {
                        blocker: BoundaryContourNestingBlocker {
                            reason: crate::UncertaintyReason::Boundary,
                            first_contour_index: candidate_index,
                            second_contour_index: container_index,
                        },
                        counts,
                    });
                }
                Classification::Uncertain(reason) => {
                    return Ok(BoundaryContourNestingOutcome::Blocked {
                        blocker: BoundaryContourNestingBlocker {
                            reason,
                            first_contour_index: candidate_index,
                            second_contour_index: container_index,
                        },
                        counts,
                    });
                }
            }
        }

        entries.push(BoundaryContourNestingEntry {
            sample_point: sample.clone(),
            containing_contour_indices,
        });
    }

    Ok(BoundaryContourNestingOutcome::Decided {
        nesting: BoundaryContourNestingDepths { entries },
        counts,
    })
}
