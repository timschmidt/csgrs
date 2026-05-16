//! Prepared borrowed query structures for repeated topology classification.
//!
//! Prepared views cache conservative broad-phase data but do not replace exact
//! topology. They skip only decided bounding-box misses and then delegate to the
//! same segment-intersection and boundary-first contour classification used by
//! ordinary contours and regions.

use hyperlattice::{Backend, DefaultBackend};

use crate::bbox::{Aabb2, aabb_decided_misses_point, decided_segment_aabb};
use crate::{
    Classification, Contour2, ContourIntersectionSet, ContourPointLocation, CurvePolicy,
    CurveResult, CurveString2, CurveStringIntersection, Point2, Region2, RegionContourIntersection,
    RegionContourKey, RegionContourRole, RegionIntersectionSet, RegionPointLocation, RegionSide,
    RegionView2, UncertaintyReason,
};

/// A borrowed curve string with cached segment and whole-string bounding boxes.
///
/// Prepared curve strings avoid rebuilding broad-phase boxes for repeated
/// topology queries. The cache never decides a contact on its own: it skips only
/// decided disjoint boxes and keeps exact line/arc intersections authoritative.
/// This mirrors the candidate-pruning role described by Bentley and Ottmann,
/// "Algorithms for Reporting and Counting Geometric Intersections" (1979),
/// while retaining the current flat pair enumeration.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedCurveStringView2<'a, B: Backend = DefaultBackend> {
    curve: &'a CurveString2<B>,
    segment_boxes: Vec<Option<Aabb2<B>>>,
    curve_box: Option<Aabb2<B>>,
}

impl<'a, B: Backend> PreparedCurveStringView2<'a, B> {
    /// Builds a prepared borrowed curve string.
    pub fn from_curve_string(curve: &'a CurveString2<B>, policy: &CurvePolicy) -> Self {
        let segment_boxes = decided_segment_boxes(curve.segments(), policy);
        let curve_box = union_all_decided_boxes(segment_boxes.iter().map(Option::as_ref), policy);

        Self {
            curve,
            segment_boxes,
            curve_box,
        }
    }

    /// Returns the borrowed source curve string.
    pub const fn curve_string(&self) -> &'a CurveString2<B> {
        self.curve
    }

    /// Returns the cached whole-curve box when every segment box was decided.
    pub const fn curve_box(&self) -> Option<&Aabb2<B>> {
        self.curve_box.as_ref()
    }

    /// Returns cached segment boxes in source segment order.
    pub fn segment_boxes(&self) -> &[Option<Aabb2<B>>] {
        &self.segment_boxes
    }

    /// Collects all nonempty segment-pair intersections against another
    /// prepared curve string.
    pub fn intersect_prepared_curve_string(
        &self,
        other: &PreparedCurveStringView2<'_, B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection<B>>> {
        crate::curve_string::intersect_curve_strings_with_cached_aabbs(
            self.curve,
            other.curve,
            &self.segment_boxes,
            &other.segment_boxes,
            policy,
        )
    }

    /// Collects all nonempty segment-pair intersections against an ordinary
    /// borrowed curve string.
    pub fn intersect_curve_string(
        &self,
        other: &CurveString2<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection<B>>> {
        let other = PreparedCurveStringView2::from_curve_string(other, policy);
        self.intersect_prepared_curve_string(&other, policy)
    }

    /// Classifies whether this prepared open curve string self-contacts.
    pub fn has_self_contacts(&self, policy: &CurvePolicy) -> CurveResult<Classification<bool>> {
        crate::self_intersect::segments_have_self_contacts_with_cached_aabbs(
            self.curve.segments(),
            &self.segment_boxes,
            false,
            policy,
        )
    }
}

/// A borrowed contour with cached contour and segment bounding boxes.
///
/// Prepared contours are useful when the same contour participates in many
/// topology queries. The cached boxes are conservative candidate filters only:
/// decided disjoint boxes skip a pair, while hits and uncertain boxes still run
/// the exact line/arc intersection code. This is the same broad-phase role that
/// Bentley and Ottmann assign to ordered geometric candidates in "Algorithms for
/// Reporting and Counting Geometric Intersections" (1979), kept here as a flat
/// pair scan until the crate grows a sweep-line index.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedContourView2<'a, B: Backend = DefaultBackend> {
    contour: &'a Contour2<B>,
    segment_boxes: Vec<Option<Aabb2<B>>>,
    contour_box: Option<Aabb2<B>>,
}

impl<'a, B: Backend> PreparedContourView2<'a, B> {
    /// Builds a prepared borrowed contour.
    pub fn from_contour(contour: &'a Contour2<B>, policy: &CurvePolicy) -> Self {
        let segment_boxes = decided_segment_boxes(contour.segments(), policy);
        let contour_box = union_all_decided_boxes(segment_boxes.iter().map(Option::as_ref), policy);

        Self {
            contour,
            segment_boxes,
            contour_box,
        }
    }

    /// Returns the borrowed source contour.
    pub const fn contour(&self) -> &'a Contour2<B> {
        self.contour
    }

    /// Returns the cached whole-contour box when every segment box was decided.
    pub const fn contour_box(&self) -> Option<&Aabb2<B>> {
        self.contour_box.as_ref()
    }

    /// Returns cached segment boxes in source segment order.
    pub fn segment_boxes(&self) -> &[Option<Aabb2<B>>] {
        &self.segment_boxes
    }

    /// Intersects two prepared contours using their cached broad-phase boxes.
    pub fn intersect_prepared_contour(
        &self,
        other: &PreparedContourView2<'_, B>,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourIntersectionSet<B>> {
        crate::events::intersect_contours_with_cached_aabbs(
            self.contour,
            other.contour,
            self.contour_box(),
            other.contour_box(),
            &self.segment_boxes,
            &other.segment_boxes,
            policy,
        )
    }

    /// Intersects this prepared contour against an ordinary borrowed contour.
    pub fn intersect_contour(
        &self,
        other: &Contour2<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourIntersectionSet<B>> {
        let other = PreparedContourView2::from_contour(other, policy);
        self.intersect_prepared_contour(&other, policy)
    }

    /// Classifies a point against this prepared contour.
    pub fn classify_point(
        &self,
        point: &Point2<B>,
        policy: &CurvePolicy,
    ) -> Classification<ContourPointLocation> {
        crate::contour::classify_contour_point_with_cached_aabbs(
            self.contour,
            point,
            self.contour_box(),
            &self.segment_boxes,
            policy,
        )
    }

    /// Returns true when the point lies on this prepared contour boundary.
    pub fn point_on_boundary(
        &self,
        point: &Point2<B>,
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        crate::contour::point_on_contour_boundary_with_cached_aabbs(
            self.contour,
            point,
            self.contour_box(),
            &self.segment_boxes,
            policy,
        )
    }

    /// Computes the winding number for a point not on this prepared boundary.
    pub fn winding_number(&self, point: &Point2<B>, policy: &CurvePolicy) -> Classification<i32> {
        crate::contour::contour_winding_number_with_cached_aabbs(
            self.contour,
            point,
            self.contour_box(),
            &self.segment_boxes,
            policy,
        )
    }

    /// Classifies whether this prepared closed contour self-contacts.
    pub fn has_self_contacts(&self, policy: &CurvePolicy) -> CurveResult<Classification<bool>> {
        crate::self_intersect::segments_have_self_contacts_with_cached_aabbs(
            self.contour.segments(),
            &self.segment_boxes,
            true,
            policy,
        )
    }
}

/// A borrowed region view with cached contour and region bounding boxes.
///
/// This is useful when many points or intersection queries are run against the
/// same region. The cached boxes are only broad-phase filters: a decided point
/// miss contributes no depth, decided disjoint contour boxes skip intersection
/// candidates, and hits or uncertain boxes still run exact topology. Build the
/// prepared view with the same policy family used for later queries so arc
/// extrema and coordinate ordering are interpreted consistently.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedRegionView2<'a, B: Backend = DefaultBackend> {
    material_contours: Vec<&'a Contour2<B>>,
    hole_contours: Vec<&'a Contour2<B>>,
    material_prepared_contours: Vec<PreparedContourView2<'a, B>>,
    hole_prepared_contours: Vec<PreparedContourView2<'a, B>>,
    region_box: Option<Aabb2<B>>,
}

impl<'a, B: Backend> PreparedRegionView2<'a, B> {
    /// Builds a prepared view from an owned region.
    pub fn from_region(region: &'a Region2<B>, policy: &CurvePolicy) -> Self {
        Self::from_region_view(&region.as_view(), policy)
    }

    /// Builds a prepared view from a borrowed region view.
    pub fn from_region_view(region: &RegionView2<'a, B>, policy: &CurvePolicy) -> Self {
        let material_contours = region.material_contours().to_vec();
        let hole_contours = region.hole_contours().to_vec();
        let material_prepared_contours = prepared_contours(&material_contours, policy);
        let hole_prepared_contours = prepared_contours(&hole_contours, policy);
        let region_box = union_all_decided_boxes(
            material_prepared_contours
                .iter()
                .chain(hole_prepared_contours.iter())
                .map(PreparedContourView2::contour_box),
            policy,
        );

        Self {
            material_contours,
            hole_contours,
            material_prepared_contours,
            hole_prepared_contours,
            region_box,
        }
    }

    /// Returns the cached whole-region box when every contour box was decided.
    pub const fn region_box(&self) -> Option<&Aabb2<B>> {
        self.region_box.as_ref()
    }

    /// Returns material contours in the prepared view.
    pub fn material_contours(&self) -> &[&'a Contour2<B>] {
        &self.material_contours
    }

    /// Returns hole contours in the prepared view.
    pub fn hole_contours(&self) -> &[&'a Contour2<B>] {
        &self.hole_contours
    }

    /// Returns prepared material contours in region-bin order.
    pub fn prepared_material_contours(&self) -> &[PreparedContourView2<'a, B>] {
        &self.material_prepared_contours
    }

    /// Returns prepared hole contours in region-bin order.
    pub fn prepared_hole_contours(&self) -> &[PreparedContourView2<'a, B>] {
        &self.hole_prepared_contours
    }

    /// Classifies a point against this prepared region view.
    pub fn classify_point(
        &self,
        point: &Point2<B>,
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

    /// Returns signed containment depth for a non-boundary point.
    ///
    /// This follows the same signed material-minus-hole convention as
    /// [`RegionView2::signed_depth`]. Decided cached-box misses are skipped, then
    /// candidate contours are classified with the boundary-first winding
    /// structure described by Hormann and Agathos, "The Point in Polygon Problem
    /// for Arbitrary Polygons" (2001), with this crate's circular-arc extension.
    pub fn signed_depth(&self, point: &Point2<B>, policy: &CurvePolicy) -> Classification<i32> {
        if self
            .region_box
            .as_ref()
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            return Classification::Decided(0);
        }

        let mut depth = 0;
        match accumulate_depth(
            &mut depth,
            &self.material_prepared_contours,
            point,
            1,
            policy,
        ) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
        match accumulate_depth(&mut depth, &self.hole_prepared_contours, point, -1, policy) {
            Classification::Decided(()) => Classification::Decided(depth),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Collects normalized topology events against another prepared region.
    ///
    /// This reuses cached contour and segment boxes for the candidate phase and
    /// then delegates candidate pairs to the same exact line/arc intersection
    /// normalization as [`RegionView2::intersect_region`]. The cache changes the
    /// amount of repeated broad-phase work, not the topology contract.
    pub fn intersect_prepared_region(
        &self,
        other: &PreparedRegionView2<'_, B>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionIntersectionSet<B>> {
        let mut pairs = Vec::new();

        collect_prepared_role_pairs(
            &mut pairs,
            &self.material_prepared_contours,
            RegionContourRole::Material,
            &other.material_prepared_contours,
            RegionContourRole::Material,
            policy,
        )?;
        collect_prepared_role_pairs(
            &mut pairs,
            &self.material_prepared_contours,
            RegionContourRole::Material,
            &other.hole_prepared_contours,
            RegionContourRole::Hole,
            policy,
        )?;
        collect_prepared_role_pairs(
            &mut pairs,
            &self.hole_prepared_contours,
            RegionContourRole::Hole,
            &other.material_prepared_contours,
            RegionContourRole::Material,
            policy,
        )?;
        collect_prepared_role_pairs(
            &mut pairs,
            &self.hole_prepared_contours,
            RegionContourRole::Hole,
            &other.hole_prepared_contours,
            RegionContourRole::Hole,
            policy,
        )?;

        Ok(RegionIntersectionSet::new(pairs))
    }

    /// Collects normalized topology events against an ordinary region view.
    pub fn intersect_region(
        &self,
        other: &RegionView2<'_, B>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionIntersectionSet<B>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.intersect_prepared_region(&other, policy)
    }
}

impl<B: Backend> CurveString2<B> {
    /// Builds a prepared borrowed curve string for repeated topology queries.
    pub fn prepare_topology_queries(
        &self,
        policy: &CurvePolicy,
    ) -> PreparedCurveStringView2<'_, B> {
        PreparedCurveStringView2::from_curve_string(self, policy)
    }
}

impl<B: Backend> Contour2<B> {
    /// Builds a prepared borrowed contour for repeated topology queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedContourView2<'_, B> {
        PreparedContourView2::from_contour(self, policy)
    }
}

impl<B: Backend> Region2<B> {
    /// Builds a prepared borrowed view for repeated point classification.
    pub fn prepare_point_classifier(&self, policy: &CurvePolicy) -> PreparedRegionView2<'_, B> {
        PreparedRegionView2::from_region(self, policy)
    }

    /// Builds a prepared borrowed view for repeated point and event queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedRegionView2<'_, B> {
        PreparedRegionView2::from_region(self, policy)
    }
}

impl<'a, B: Backend> RegionView2<'a, B> {
    /// Builds a prepared borrowed view for repeated point classification.
    pub fn prepare_point_classifier(&self, policy: &CurvePolicy) -> PreparedRegionView2<'a, B> {
        PreparedRegionView2::from_region_view(self, policy)
    }

    /// Builds a prepared borrowed view for repeated point and event queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedRegionView2<'a, B> {
        PreparedRegionView2::from_region_view(self, policy)
    }
}

fn prepared_contours<'a, B: Backend>(
    contours: &[&'a Contour2<B>],
    policy: &CurvePolicy,
) -> Vec<PreparedContourView2<'a, B>>
where
    B: 'a,
{
    contours
        .iter()
        .map(|contour| PreparedContourView2::from_contour(contour, policy))
        .collect()
}

fn decided_segment_boxes<B: Backend>(
    segments: &[crate::Segment2<B>],
    policy: &CurvePolicy,
) -> Vec<Option<Aabb2<B>>> {
    segments
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect()
}

fn union_all_decided_boxes<'a, B: Backend, I>(boxes: I, policy: &CurvePolicy) -> Option<Aabb2<B>>
where
    I: IntoIterator<Item = Option<&'a Aabb2<B>>>,
    B: 'a,
{
    let mut boxes = boxes.into_iter();
    let first = boxes.next()??.clone();
    let mut merged = first;

    for bbox in boxes {
        let bbox = bbox?;
        let Classification::Decided(next) = merged.union(bbox, policy) else {
            return None;
        };
        merged = next;
    }

    Some(merged)
}

fn accumulate_depth<B: Backend>(
    depth: &mut i32,
    contours: &[PreparedContourView2<'_, B>],
    point: &Point2<B>,
    sign: i32,
    policy: &CurvePolicy,
) -> Classification<()> {
    for contour in contours {
        if contour
            .contour_box()
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            continue;
        }

        match contour.classify_point(point, policy) {
            Classification::Decided(ContourPointLocation::Inside) => *depth += sign,
            Classification::Decided(ContourPointLocation::Outside) => {}
            Classification::Decided(ContourPointLocation::Boundary) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }

    Classification::Decided(())
}

fn collect_prepared_role_pairs<B: Backend>(
    pairs: &mut Vec<RegionContourIntersection<B>>,
    first_contours: &[PreparedContourView2<'_, B>],
    first_role: RegionContourRole,
    second_contours: &[PreparedContourView2<'_, B>],
    second_role: RegionContourRole,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    for (first_index, first_contour) in first_contours.iter().enumerate() {
        for (second_index, second_contour) in second_contours.iter().enumerate() {
            let intersections = first_contour.intersect_prepared_contour(second_contour, policy)?;
            if intersections.is_empty() {
                continue;
            }

            pairs.push(RegionContourIntersection {
                first: RegionContourKey::new(RegionSide::First, first_role, first_index),
                second: RegionContourKey::new(RegionSide::Second, second_role, second_index),
                intersections,
            });
        }
    }

    Ok(())
}
