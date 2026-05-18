//! Planar regions assembled from signed closed contours.

use std::cmp::Ordering;

use crate::bbox::{Aabb2, aabb_decided_misses_point, decided_contour_aabb};
use crate::classify::compare_reals;
use crate::{
    Classification, Contour2, ContourPointLocation, CurvePolicy, CurveResult, Point2, Real,
    RegionAreaContourRole, RegionAreaUnsupportedContour2, RegionFilledAreaReport2,
    UncertaintyReason,
};

/// Point location relative to a planar region.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionPointLocation {
    /// The point is outside the filled region.
    Outside,
    /// The point lies on a material or hole boundary.
    Boundary,
    /// The point is inside the filled region.
    Inside,
}

/// An owned planar region with explicit material and hole contour bins.
///
/// The bins are signed by role, not by trusting contour orientation. A point's
/// depth is the number of containing material contours minus the number of
/// containing hole contours. Positive depth is inside; zero or negative depth
/// is outside. This intentionally supports nested islands by putting the inner
/// island contour back in the material bin.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Region2 {
    material_contours: Vec<Contour2>,
    hole_contours: Vec<Contour2>,
}

/// A material contour and the hole contours owned by it.
///
/// This is a borrowed topology view over [`Region2`] / [`RegionView2`]. It
/// keeps hole ownership in hypercurve rather than forcing downstream crates to
/// project contours to sampled rings before grouping them.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionContourProfile<'a> {
    /// Filled/material boundary contour.
    pub material: &'a Contour2,
    /// Subtractive hole contours classified inside `material`.
    pub holes: Vec<&'a Contour2>,
}

impl Region2 {
    /// Constructs an empty region.
    pub const fn empty() -> Self {
        Self {
            material_contours: Vec::new(),
            hole_contours: Vec::new(),
        }
    }

    /// Constructs a region from explicit material and hole contour bins.
    pub const fn new(material_contours: Vec<Contour2>, hole_contours: Vec<Contour2>) -> Self {
        Self {
            material_contours,
            hole_contours,
        }
    }

    /// Constructs a region from material contours only.
    pub const fn from_material_contours(material_contours: Vec<Contour2>) -> Self {
        Self {
            material_contours,
            hole_contours: Vec::new(),
        }
    }

    /// Returns material contours.
    pub fn material_contours(&self) -> &[Contour2] {
        &self.material_contours
    }

    /// Returns hole contours.
    pub fn hole_contours(&self) -> &[Contour2] {
        &self.hole_contours
    }

    /// Returns true when the region has no contours.
    pub fn is_empty(&self) -> bool {
        self.material_contours.is_empty() && self.hole_contours.is_empty()
    }

    /// Returns a borrowed view over this region.
    pub fn as_view(&self) -> RegionView2<'_> {
        RegionView2::new(&self.material_contours, &self.hole_contours)
    }

    /// Classifies a point against this region.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<RegionPointLocation> {
        self.as_view().classify_point(point, policy)
    }

    /// Returns signed containment depth for non-boundary points.
    pub fn signed_depth(&self, point: &Point2, policy: &CurvePolicy) -> Classification<i32> {
        self.as_view().signed_depth(point, policy)
    }

    /// Returns the exact filled area implied by the region's material/hole roles.
    ///
    /// Unlike [`Contour2::signed_area`], this ignores contour orientation:
    /// material bins add the absolute contour area and hole bins subtract it.
    /// This keeps the region role model explicit and avoids treating winding as
    /// hidden topology state. The area is accumulated from exact
    /// Green's-theorem contour facts and branches only after the sign of each
    /// contour contribution is certified, following Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    ///
    /// Returns `Decided(None)` when a contour contains a segment whose exact
    /// area contribution is not implemented by the current object model.
    pub fn filled_area(&self, policy: &CurvePolicy) -> CurveResult<Classification<Option<Real>>> {
        self.as_view().filled_area(policy)
    }

    /// Returns an auditable filled-area report for this region.
    ///
    /// The report carries role-normalized material and hole totals, the policy
    /// used for exact sign certification, and unsupported contour details.
    /// This is the certificate-bearing counterpart to [`Region2::filled_area`].
    pub fn filled_area_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RegionFilledAreaReport2>> {
        self.as_view().filled_area_report(policy)
    }

    /// Groups material contours with the hole contours they contain.
    ///
    /// Ownership is decided with exact contour point classification before any
    /// finite export projection exists. This follows the boundary-first
    /// point-in-polygon structure surveyed by Hormann and Agathos, "The Point
    /// in Polygon Problem for Arbitrary Polygons," *Computational Geometry*
    /// 20(3), 2001 (<https://doi.org/10.1016/S0925-7721(01)00012-8>), and the
    /// exact-object/API-boundary split advocated by Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn contour_profiles(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<Vec<RegionContourProfile<'_>>> {
        contour_profiles_from_iter(
            self.material_contours.iter(),
            self.hole_contours.iter(),
            policy,
        )
    }

    /// Collects normalized topology events against another region.
    pub fn intersect_region(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> crate::CurveResult<crate::RegionIntersectionSet> {
        self.as_view().intersect_region(&other.as_view(), policy)
    }
}

/// Borrowed view over material and hole contours.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionView2<'a> {
    material_contours: Vec<&'a Contour2>,
    hole_contours: Vec<&'a Contour2>,
}

impl<'a> RegionView2<'a> {
    /// Constructs a borrowed view from explicit material and hole contour slices.
    pub fn new(material_contours: &'a [Contour2], hole_contours: &'a [Contour2]) -> Self {
        Self::from_contours(material_contours, hole_contours)
    }

    /// Constructs a borrowed view from arbitrary borrowed contour iterators.
    pub fn from_contours<I, J>(material_contours: I, hole_contours: J) -> Self
    where
        I: IntoIterator<Item = &'a Contour2>,
        J: IntoIterator<Item = &'a Contour2>,
    {
        Self {
            material_contours: material_contours.into_iter().collect(),
            hole_contours: hole_contours.into_iter().collect(),
        }
    }

    /// Returns material contours.
    pub fn material_contours(&self) -> &[&'a Contour2] {
        &self.material_contours
    }

    /// Returns hole contours.
    pub fn hole_contours(&self) -> &[&'a Contour2] {
        &self.hole_contours
    }

    /// Returns true when the view has no contours.
    pub fn is_empty(&self) -> bool {
        self.material_contours.is_empty() && self.hole_contours.is_empty()
    }

    /// Classifies a point against this region view.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<RegionPointLocation> {
        let depth = match self.signed_depth(point, policy) {
            Classification::Decided(depth) => depth,
            Classification::Uncertain(UncertaintyReason::Boundary) => {
                return Classification::Decided(RegionPointLocation::Boundary);
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        Classification::Decided(if depth > 0 {
            RegionPointLocation::Inside
        } else {
            RegionPointLocation::Outside
        })
    }

    /// Returns signed containment depth for non-boundary points.
    ///
    /// Boundary points are reported as `RegionPointLocation::Boundary` through
    /// [`RegionView2::classify_point`] and as `Uncertain(Boundary)` here through
    /// contour-level classification propagation. Decided bounding-box misses
    /// skip exact contour classification; this keeps the standard
    /// boundary-first winding structure from Hormann and Agathos, "The Point in
    /// Polygon Problem for Arbitrary Polygons" (2001), while avoiding work for
    /// sparse material/hole bins.
    pub fn signed_depth(&self, point: &Point2, policy: &CurvePolicy) -> Classification<i32> {
        if let Ok(Classification::Decided(region_bbox)) = Aabb2::from_region_view(self, policy)
            && aabb_decided_misses_point(&region_bbox, point, policy)
        {
            return Classification::Decided(0);
        }

        let mut depth = 0;

        for contour in &self.material_contours {
            if contour_aabb_misses_point(contour, point, policy) {
                continue;
            }

            match contour.classify_point(point, policy) {
                Classification::Decided(ContourPointLocation::Inside) => depth += 1,
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                }
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }

        for contour in &self.hole_contours {
            if contour_aabb_misses_point(contour, point, policy) {
                continue;
            }

            match contour.classify_point(point, policy) {
                Classification::Decided(ContourPointLocation::Inside) => depth -= 1,
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                }
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }

        Classification::Decided(depth)
    }

    /// Returns the exact filled area implied by this borrowed region view.
    ///
    /// Material contours add their certified absolute area and hole contours
    /// subtract theirs. This mirrors [`Region2::filled_area`] for borrowed
    /// region data without cloning contour bins.
    pub fn filled_area(&self, policy: &CurvePolicy) -> CurveResult<Classification<Option<Real>>> {
        Ok(self
            .filled_area_report(policy)?
            .map(|report| report.filled_area))
    }

    /// Returns an auditable filled-area report for this borrowed region view.
    ///
    /// Green's-theorem contour areas are normalized by region role, not by
    /// winding orientation. Each contour's sign is used only after exact
    /// ordering has been certified by the active policy, matching Yap's
    /// "Towards Exact Geometric Computation" (1997) rule that geometric
    /// branching depends on certified predicates.
    pub fn filled_area_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RegionFilledAreaReport2>> {
        let mut report = RegionFilledAreaReport2 {
            filled_area: Some(Real::zero()),
            material_area: Real::zero(),
            hole_area: Real::zero(),
            material_contour_count: self.material_contours.len(),
            hole_contour_count: self.hole_contours.len(),
            unsupported_contours: Vec::new(),
            construction_policy: policy.clone(),
        };

        for (index, contour) in self.material_contours.iter().enumerate() {
            if let Classification::Uncertain(reason) = accumulate_contour_role_area(
                &mut report,
                RegionAreaContourRole::Material,
                index,
                contour,
                policy,
            )? {
                return Ok(Classification::Uncertain(reason));
            }
        }
        for (index, contour) in self.hole_contours.iter().enumerate() {
            if let Classification::Uncertain(reason) = accumulate_contour_role_area(
                &mut report,
                RegionAreaContourRole::Hole,
                index,
                contour,
                policy,
            )? {
                return Ok(Classification::Uncertain(reason));
            }
        }

        report.filled_area = if report.unsupported_contours.is_empty() {
            Some(&report.material_area - &report.hole_area)
        } else {
            None
        };

        Ok(Classification::Decided(report))
    }

    /// Groups material contours with the hole contours they contain.
    ///
    /// The representative point is taken from the hole boundary and classified
    /// against every material contour with the same certified contour
    /// classifier used by region containment. A boundary result is accepted as
    /// ownership so shared/degenerate finite projections do not silently switch
    /// to centroid heuristics. If classification is uncertain, the uncertainty
    /// is returned to the caller rather than assigning a hole arbitrarily.
    pub fn contour_profiles(
        &'a self,
        policy: &CurvePolicy,
    ) -> Classification<Vec<RegionContourProfile<'a>>> {
        contour_profiles_from_iter(
            self.material_contours.iter().copied(),
            self.hole_contours.iter().copied(),
            policy,
        )
    }

    /// Collects normalized topology events against another region view.
    pub fn intersect_region(
        &self,
        other: &RegionView2<'_>,
        policy: &CurvePolicy,
    ) -> crate::CurveResult<crate::RegionIntersectionSet> {
        crate::region_events::intersect_region_views(self, other, policy)
    }
}

fn contour_aabb_misses_point(contour: &Contour2, point: &Point2, policy: &CurvePolicy) -> bool {
    decided_contour_aabb(contour, policy)
        .as_ref()
        .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
}

fn contour_profiles_from_iter<'a>(
    material_contours: impl IntoIterator<Item = &'a Contour2>,
    hole_contours: impl IntoIterator<Item = &'a Contour2>,
    policy: &CurvePolicy,
) -> Classification<Vec<RegionContourProfile<'a>>> {
    let mut profiles = material_contours
        .into_iter()
        .map(|material| RegionContourProfile {
            material,
            holes: Vec::new(),
        })
        .collect::<Vec<_>>();
    let holes = hole_contours.into_iter().collect::<Vec<_>>();

    if profiles.is_empty() {
        return if holes.is_empty() {
            Classification::Decided(profiles)
        } else {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        };
    }

    for hole in holes {
        let Some(point) = hole.segments().first().map(|segment| segment.start()) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let mut owner = None;
        for (index, profile) in profiles.iter().enumerate() {
            match profile.material.classify_point(point, policy) {
                Classification::Decided(
                    ContourPointLocation::Inside | ContourPointLocation::Boundary,
                ) => {
                    owner = Some(index);
                    break;
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }

        let Some(owner) = owner else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        profiles[owner].holes.push(hole);
    }

    Classification::Decided(profiles)
}

fn accumulate_contour_role_area(
    report: &mut RegionFilledAreaReport2,
    role: RegionAreaContourRole,
    contour_index: usize,
    contour: &Contour2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    let contour_report = contour.signed_area_report()?;
    let Some(area) = contour_report.signed_area.clone() else {
        report
            .unsupported_contours
            .push(RegionAreaUnsupportedContour2 {
                role,
                contour_index,
                contour_report,
            });
        return Ok(Classification::Decided(()));
    };

    let area = match certified_absolute_area(area, policy)? {
        Classification::Decided(area) => area,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    match role {
        RegionAreaContourRole::Material => report.material_area = &report.material_area + &area,
        RegionAreaContourRole::Hole => report.hole_area = &report.hole_area + &area,
    }

    Ok(Classification::Decided(()))
}

fn certified_absolute_area(area: Real, policy: &CurvePolicy) -> CurveResult<Classification<Real>> {
    match compare_reals(&area, &Real::zero(), policy) {
        Some(Ordering::Less) => Ok(Classification::Decided(Real::zero() - &area)),
        Some(Ordering::Equal | Ordering::Greater) => Ok(Classification::Decided(area)),
        None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}
