//! Planar regions assembled from signed closed contours.

use crate::bbox::{Aabb2, aabb_decided_misses_point, decided_contour_aabb};
use crate::{
    Classification, Contour2, ContourPointLocation, CurveContext, Point2, UncertaintyReason,
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
pub(crate) struct LineArcRegion2 {
    material_contours: Vec<Contour2>,
    hole_contours: Vec<Contour2>,
}

impl LineArcRegion2 {
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

    /// Returns material contours.
    pub fn material_contours(&self) -> &[Contour2] {
        &self.material_contours
    }

    /// Returns hole contours.
    pub fn hole_contours(&self) -> &[Contour2] {
        &self.hole_contours
    }

    /// Returns a borrowed view over this region.
    pub fn as_view(&self) -> RegionView2<'_> {
        RegionView2::new(&self.material_contours, &self.hole_contours)
    }

    /// Classifies a point against this region.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> Classification<RegionPointLocation> {
        self.as_view().classify_point(point, policy)
    }

    /// Classifies a batch of points while building exact query indexes once.
    pub fn classify_points(
        &self,
        points: &[Point2],
        policy: &CurveContext,
    ) -> Vec<Classification<RegionPointLocation>> {
        self.as_view().classify_points(points, policy)
    }

    /// Returns conservative structural facts for this region immediately.
    pub fn structural_facts(&self, policy: &CurveContext) -> crate::RegionFacts {
        self.as_view().structural_facts(policy)
    }

    /// Returns signed containment depth for non-boundary points.
    pub fn signed_depth(&self, point: &Point2, policy: &CurveContext) -> Classification<i32> {
        self.as_view().signed_depth(point, policy)
    }
}

/// Borrowed view over material and hole contours.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct RegionView2<'a> {
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

    /// Classifies a point against this region view.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurveContext,
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

    /// Classifies a batch of points through one immediate exact region pass.
    ///
    /// Region, contour, segment, winding, and predicate indexes are temporary
    /// implementation details shared by every point in this call.
    pub fn classify_points(
        &self,
        points: &[Point2],
        policy: &CurveContext,
    ) -> Vec<Classification<RegionPointLocation>> {
        let index = crate::prepared::RegionQuery2::from_region_view(self, policy);
        points
            .iter()
            .map(|point| index.classify_point(point, policy))
            .collect()
    }

    /// Returns conservative structural facts for this borrowed region immediately.
    pub fn structural_facts(&self, policy: &CurveContext) -> crate::RegionFacts {
        crate::prepared::region_view_facts(self, policy)
    }

    /// Returns signed containment depth for non-boundary points.
    ///
    /// Boundary points are reported as `RegionPointLocation::Boundary` through
    /// [`RegionView2::classify_point`] and as `Uncertain(Boundary)` here through
    /// contour-level classification propagation. Decided bounding-box misses
    /// skip exact contour classification; this keeps the standard
    /// boundary-first winding structure from boundary-first winding classification, while avoiding work for
    /// sparse material/hole bins.
    pub fn signed_depth(&self, point: &Point2, policy: &CurveContext) -> Classification<i32> {
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
}

fn contour_aabb_misses_point(contour: &Contour2, point: &Point2, policy: &CurveContext) -> bool {
    decided_contour_aabb(contour, policy)
        .as_ref()
        .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
}
