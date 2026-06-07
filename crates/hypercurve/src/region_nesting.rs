//! Contour nesting and material/hole role assignment.
//!
//! This module turns already-closed boundary contours into the signed contour
//! bins used by [`crate::Region2`]. It assumes intersections and overlaps have
//! already been resolved by earlier topology stages.

use crate::{
    Classification, Contour2, ContourPointLocation, CurveError, CurvePolicy, CurveResult, Region2,
};

#[derive(Clone, Debug, Eq, PartialEq)]
struct BoundaryContourNestingDepths {
    depths: Vec<usize>,
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

        Ok(Classification::Decided(Region2::new(
            material_contours,
            hole_contours,
        )))
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

    let mut depths = Vec::with_capacity(contours.len());

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
    }))
}
