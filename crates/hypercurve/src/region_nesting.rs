//! Contour nesting and material/hole role assignment.
//!
//! This module turns already-closed boundary contours into the signed contour
//! bins used by [`crate::Region2`]. It assumes intersections and overlaps have
//! already been resolved by earlier topology stages.

use crate::{
    Classification, Contour2, ContourPointLocation, CurveError, CurvePolicy, CurveResult, FillRule,
    Point2, Region2, RetainedTopologyStatus, UncertaintyReason,
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

#[derive(Clone, Debug, PartialEq)]
struct BoundaryContourNestingDepths {
    entries: Vec<BoundaryContourNestingEntry>,
}

#[derive(Clone, Debug, PartialEq)]
struct BoundaryContourNestingEntry {
    sample_point: Point2,
    containing_contour_indices: Vec<usize>,
}

impl Region2 {
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
        let nesting = match contour_nesting_depths(&contours, policy)? {
            Classification::Decided(nesting) => nesting,
            Classification::Uncertain(reason) => {
                return Ok(blocked_boundary_contour_region_result(
                    source_contour_count,
                    source_segment_count,
                    retained_status_for_boundary_contour_blocker(reason),
                    reason,
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

fn blocked_boundary_contour_region_result(
    source_contour_count: usize,
    source_segment_count: usize,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> RegionBoundaryContourBuildResult2 {
    RegionBoundaryContourBuildResult2 {
        region: None,
        report: RegionBoundaryContourBuildReport2 {
            stage: RegionBoundaryContourBuildStage2::NestingValidation,
            source_contour_count,
            source_segment_count,
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
) -> CurveResult<Classification<BoundaryContourNestingDepths>> {
    for (left_index, left) in contours.iter().enumerate() {
        for right in &contours[left_index + 1..] {
            if !left.intersect_contour(right, policy)?.is_empty() {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Boundary,
                ));
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

            match container.classify_point(sample, policy) {
                Classification::Decided(ContourPointLocation::Inside) => {
                    containing_contour_indices.push(container_index);
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(Classification::Uncertain(
                        crate::UncertaintyReason::Boundary,
                    ));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }

        entries.push(BoundaryContourNestingEntry {
            sample_point: sample.clone(),
            containing_contour_indices,
        });
    }

    Ok(Classification::Decided(BoundaryContourNestingDepths {
        entries,
    }))
}
