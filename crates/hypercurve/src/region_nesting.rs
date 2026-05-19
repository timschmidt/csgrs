//! Contour nesting and material/hole role assignment.
//!
//! This module turns already-closed boundary contours into the signed contour
//! bins used by [`crate::Region2`]. It assumes intersections and overlaps have
//! already been resolved by earlier topology stages.

use crate::{
    Classification, Contour2, ContourPointLocation, CurveError, CurvePolicy, CurveResult, Region2,
};

/// Exact role-assignment status for boundary-contour region reconstruction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoundaryContourNestingStatus {
    /// No boundary contours were provided, so the reconstructed region is empty.
    Empty,
    /// Every contour received a material or hole role from exact containment
    /// classifications.
    Valid,
}

impl BoundaryContourNestingStatus {
    /// Returns true when the report certifies a usable role assignment.
    pub const fn is_valid(self) -> bool {
        matches!(self, Self::Empty | Self::Valid)
    }
}

/// Exact audit report for boundary-contour material/hole role assignment.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BoundaryContourNestingAuditReport2 {
    /// Final nesting audit status.
    pub status: BoundaryContourNestingStatus,
    /// Number of input boundary contours considered.
    pub input_contour_count: usize,
    /// Number of contour-vs-contour containment classifications replayed.
    pub checked_containment_pair_count: usize,
    /// Number of contours assigned to the material bin.
    pub material_contour_count: usize,
    /// Number of contours assigned to the hole bin.
    pub hole_contour_count: usize,
    /// Even-odd containment depth for each input contour in input order.
    pub contour_depths: Vec<usize>,
}

impl BoundaryContourNestingAuditReport2 {
    /// Returns true when the report certifies a usable role assignment.
    pub const fn is_valid(&self) -> bool {
        self.status.is_valid()
    }
}

/// Report-bearing result of reconstructing a region from boundary contours.
#[derive(Clone, Debug, PartialEq)]
pub struct BoundaryContourNestingReport2 {
    /// Role-assigned reconstructed region.
    pub result: Region2,
    /// Exact audit over the containment classifications used for role assignment.
    pub audit: BoundaryContourNestingAuditReport2,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct BoundaryContourNestingDepths {
    depths: Vec<usize>,
    checked_containment_pair_count: usize,
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
        match contours_to_nested_region_report(contours, policy)? {
            Classification::Decided(report) => Ok(Classification::Decided(report.result)),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Builds a region from boundary contours and reports the exact nesting audit.
    ///
    /// This is the certificate-bearing counterpart to
    /// [`Region2::from_boundary_contours`].  Role assignment is reduced to exact
    /// point-in-contour classifications between every candidate contour and
    /// every possible container.  Hormann and Agathos describe the point-in-
    /// polygon classification problem and its boundary degeneracies
    /// (K. Hormann and A. Agathos, "The point in polygon problem for arbitrary
    /// polygons," *Computational Geometry* 20(3), 2001); this method exposes
    /// the replay count and preserves boundary hits as
    /// [`Classification::Uncertain`] instead of silently choosing a role.  That
    /// matches Yap's requirement that exact geometric computation certify the
    /// combinatorial object, not only its scalar coordinates.
    pub fn from_boundary_contours_report(
        contours: Vec<Contour2>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BoundaryContourNestingReport2>> {
        contours_to_nested_region_report(contours, policy)
    }
}

pub(crate) fn contours_to_nested_region_report(
    contours: Vec<Contour2>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BoundaryContourNestingReport2>> {
    let input_contour_count = contours.len();
    let nesting = match contour_nesting_depths(&contours, policy)? {
        Classification::Decided(nesting) => nesting,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut material_contours = Vec::new();
    let mut hole_contours = Vec::new();

    for (contour, depth) in contours.into_iter().zip(nesting.depths.iter().copied()) {
        if depth % 2 == 0 {
            material_contours.push(contour);
        } else {
            hole_contours.push(contour);
        }
    }

    let material_contour_count = material_contours.len();
    let hole_contour_count = hole_contours.len();
    let result = Region2::new(material_contours, hole_contours);

    Ok(Classification::Decided(BoundaryContourNestingReport2 {
        result,
        audit: BoundaryContourNestingAuditReport2 {
            status: if input_contour_count == 0 {
                BoundaryContourNestingStatus::Empty
            } else {
                BoundaryContourNestingStatus::Valid
            },
            input_contour_count,
            checked_containment_pair_count: nesting.checked_containment_pair_count,
            material_contour_count,
            hole_contour_count,
            contour_depths: nesting.depths,
        },
    }))
}

fn contour_nesting_depths(
    contours: &[Contour2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<BoundaryContourNestingDepths>> {
    let mut depths = Vec::with_capacity(contours.len());
    let mut checked_containment_pair_count = 0_usize;

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
        let mut depth = 0_usize;

        for (container_index, container) in contours.iter().enumerate() {
            if candidate_index == container_index {
                continue;
            }

            checked_containment_pair_count += 1;
            match container.classify_point(sample, policy) {
                Classification::Decided(ContourPointLocation::Inside) => depth += 1,
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(Classification::Uncertain(
                        crate::UncertaintyReason::Boundary,
                    ));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }

        depths.push(depth);
    }

    Ok(Classification::Decided(BoundaryContourNestingDepths {
        depths,
        checked_containment_pair_count,
    }))
}
