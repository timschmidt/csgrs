//! Prepared borrowed query structures for repeated topology classification.
//!
//! Prepared views cache conservative broad-phase data but do not replace exact
//! topology. They skip only decided bounding-box misses and then delegate to the
//! same segment-intersection and boundary-first contour classification used by
//! ordinary contours and regions. This keeps preparation in the candidate
//! generation role of Bentley and Ottmann's intersection-reporting framework,
//! while preserving Shewchuk-style certified predicates for topology branches.

use crate::bbox::{Aabb2, aabb_decided_misses_point, decided_segment_aabb};
use crate::facts::{CurveStringFacts, RegionFacts};
use crate::{
    BooleanBoundaryLoopSet, BooleanOp, Classification, Contour2, ContourIntersectionSet,
    ContourPointLocation, CurvePolicy, CurveResult, CurveString2, CurveStringIntersection,
    FillRule, Point2, Region2, RegionContourIntersection, RegionContourKey, RegionContourRole,
    RegionIntersectionSet, RegionPointLocation, RegionSide, RegionView2, UncertaintyReason,
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
pub struct PreparedCurveStringView2<'a> {
    curve: &'a CurveString2,
    segment_boxes: Vec<Option<Aabb2>>,
    curve_box: Option<Aabb2>,
    facts: CurveStringFacts,
}

impl<'a> PreparedCurveStringView2<'a> {
    /// Builds a prepared borrowed curve string.
    pub fn from_curve_string(curve: &'a CurveString2, policy: &CurvePolicy) -> Self {
        // Structural-dispatch note: this preparation pass already visits every
        // segment. It is the natural place to retain facts such as all-line,
        // all-axis-aligned, monotone parameter ranges, or certified disjoint
        // interval buckets so later intersection queries can choose sweep-line
        // or grid-index paths instead of the current flat candidate scan.
        let segment_boxes = decided_segment_boxes(curve.segments(), policy);
        let curve_box = union_all_decided_boxes(segment_boxes.iter().map(Option::as_ref), policy);
        let facts = crate::facts::curve_string_facts(
            curve,
            segment_boxes.iter().filter(|bbox| bbox.is_some()).count(),
            curve_box.is_some(),
        );

        Self {
            curve,
            segment_boxes,
            curve_box,
            facts,
        }
    }

    /// Returns the borrowed source curve string.
    pub const fn curve_string(&self) -> &'a CurveString2 {
        self.curve
    }

    /// Returns the cached whole-curve box when every segment box was decided.
    pub const fn curve_box(&self) -> Option<&Aabb2> {
        self.curve_box.as_ref()
    }

    /// Returns cached segment boxes in source segment order.
    pub fn segment_boxes(&self) -> &[Option<Aabb2>] {
        &self.segment_boxes
    }

    /// Returns conservative structural facts collected while preparing.
    ///
    /// Structural-dispatch note: these facts are the intended home for future
    /// all-line, axis-aligned, common-scale, and symbolic-family routing of
    /// repeated curve-string intersection workloads. They do not certify
    /// topology; exact predicates and explicit uncertainty still do that.
    pub const fn facts(&self) -> &CurveStringFacts {
        &self.facts
    }

    /// Collects all nonempty segment-pair intersections against another
    /// prepared curve string.
    pub fn intersect_prepared_curve_string(
        &self,
        other: &PreparedCurveStringView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection>> {
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
        other: &CurveString2,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection>> {
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
pub struct PreparedContourView2<'a> {
    contour: &'a Contour2,
    segment_boxes: Vec<Option<Aabb2>>,
    contour_box: Option<Aabb2>,
    facts: CurveStringFacts,
}

impl<'a> PreparedContourView2<'a> {
    /// Builds a prepared borrowed contour.
    pub fn from_contour(contour: &'a Contour2, policy: &CurvePolicy) -> Self {
        // Structural-dispatch note: contour preparation can preserve ring-level
        // facts such as convexity, orientation certainty, y-monotonicity, and
        // hole/material provenance for future triangulation and Boolean-region
        // dispatch without weakening the exact boundary classifiers.
        let segment_boxes = decided_segment_boxes(contour.segments(), policy);
        let contour_box = union_all_decided_boxes(segment_boxes.iter().map(Option::as_ref), policy);
        let facts = crate::facts::contour_facts(
            contour,
            segment_boxes.iter().filter(|bbox| bbox.is_some()).count(),
            contour_box.is_some(),
        );

        Self {
            contour,
            segment_boxes,
            contour_box,
            facts,
        }
    }

    /// Returns the borrowed source contour.
    pub const fn contour(&self) -> &'a Contour2 {
        self.contour
    }

    /// Returns the cached whole-contour box when every segment box was decided.
    pub const fn contour_box(&self) -> Option<&Aabb2> {
        self.contour_box.as_ref()
    }

    /// Returns cached segment boxes in source segment order.
    pub fn segment_boxes(&self) -> &[Option<Aabb2>] {
        &self.segment_boxes
    }

    /// Returns conservative structural facts collected while preparing.
    ///
    /// These facts are advisory scheduling metadata in Yap's object layer:
    /// Boolean and containment code can select specialized exact paths from
    /// them, but they are not a geometric decision by themselves.
    pub const fn facts(&self) -> &CurveStringFacts {
        &self.facts
    }

    /// Intersects two prepared contours using their cached broad-phase boxes.
    pub fn intersect_prepared_contour(
        &self,
        other: &PreparedContourView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourIntersectionSet> {
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
        other: &Contour2,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourIntersectionSet> {
        let other = PreparedContourView2::from_contour(other, policy);
        self.intersect_prepared_contour(&other, policy)
    }

    /// Collects self-intersection events using this contour's cached boxes.
    pub fn intersect_self(&self, policy: &CurvePolicy) -> CurveResult<ContourIntersectionSet> {
        crate::events::intersect_contour_self_with_cached_aabbs(
            self.contour,
            &self.segment_boxes,
            policy,
        )
    }

    /// Classifies a point against this prepared contour.
    pub fn classify_point(
        &self,
        point: &Point2,
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
    pub fn point_on_boundary(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        crate::contour::point_on_contour_boundary_with_cached_aabbs(
            self.contour,
            point,
            self.contour_box(),
            &self.segment_boxes,
            policy,
        )
    }

    /// Computes the winding number for a point not on this prepared boundary.
    pub fn winding_number(&self, point: &Point2, policy: &CurvePolicy) -> Classification<i32> {
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
pub struct PreparedRegionView2<'a> {
    material_contours: Vec<&'a Contour2>,
    hole_contours: Vec<&'a Contour2>,
    material_prepared_contours: Vec<PreparedContourView2<'a>>,
    hole_prepared_contours: Vec<PreparedContourView2<'a>>,
    region_box: Option<Aabb2>,
    facts: RegionFacts,
}

impl<'a> PreparedRegionView2<'a> {
    /// Builds a prepared view from an owned region.
    pub fn from_region(region: &'a Region2, policy: &CurvePolicy) -> Self {
        Self::from_region_view(&region.as_view(), policy)
    }

    /// Builds a prepared view from a borrowed region view.
    pub fn from_region_view(region: &RegionView2<'a>, policy: &CurvePolicy) -> Self {
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
        let facts = crate::facts::region_view_facts(region, region_box.is_some());

        Self {
            material_contours,
            hole_contours,
            material_prepared_contours,
            hole_prepared_contours,
            region_box,
            facts,
        }
    }

    /// Returns the cached whole-region box when every contour box was decided.
    pub const fn region_box(&self) -> Option<&Aabb2> {
        self.region_box.as_ref()
    }

    /// Returns material contours in the prepared view.
    pub fn material_contours(&self) -> &[&'a Contour2] {
        &self.material_contours
    }

    /// Returns hole contours in the prepared view.
    pub fn hole_contours(&self) -> &[&'a Contour2] {
        &self.hole_contours
    }

    /// Reconstructs a borrowed ordinary region view over the same contours.
    ///
    /// The returned view is cheap and keeps the same contour lifetimes. It is
    /// useful when an algorithm still needs the canonical `RegionView2` shape
    /// for splitting or cloning, while prepared classifiers supply repeated
    /// point and event queries.
    pub fn as_region_view(&self) -> RegionView2<'a> {
        RegionView2::from_contours(
            self.material_contours.iter().copied(),
            self.hole_contours.iter().copied(),
        )
    }

    /// Returns prepared material contours in region-bin order.
    pub fn prepared_material_contours(&self) -> &[PreparedContourView2<'a>] {
        &self.material_prepared_contours
    }

    /// Returns prepared hole contours in region-bin order.
    pub fn prepared_hole_contours(&self) -> &[PreparedContourView2<'a>] {
        &self.hole_prepared_contours
    }

    /// Returns conservative structural facts collected while preparing.
    ///
    /// Structural-dispatch note: this is where future region-level convexity,
    /// contour orientation certainty, all-line/all-arc partitioning, common
    /// scales, and symbolic dependencies should be shared with Boolean and
    /// containment algorithms without leaking scalar representation details.
    pub const fn facts(&self) -> &RegionFacts {
        &self.facts
    }

    /// Classifies a point against this prepared region view.
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

    /// Returns signed containment depth for a non-boundary point.
    ///
    /// This follows the same signed material-minus-hole convention as
    /// [`RegionView2::signed_depth`]. Decided cached-box misses are skipped, then
    /// candidate contours are classified with the boundary-first winding
    /// structure described by Hormann and Agathos, "The Point in Polygon Problem
    /// for Arbitrary Polygons" (2001), with this crate's circular-arc extension.
    pub fn signed_depth(&self, point: &Point2, policy: &CurvePolicy) -> Classification<i32> {
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
        other: &PreparedRegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionIntersectionSet> {
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
        other: &RegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionIntersectionSet> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.intersect_prepared_region(&other, policy)
    }

    /// Computes closed boolean boundary loops against another prepared region.
    ///
    /// This prepared path runs the same split, classify, and boundary-chain
    /// traversal as [`RegionView2::boolean_boundary_loops`], but reuses cached
    /// region/contour boxes during event collection and fragment midpoint
    /// classification. Greiner and Hormann describe closed boundary traversal
    /// after intersection insertion and entry/exit classification (G. Greiner
    /// and K. Hormann, "Efficient clipping of arbitrary polygons," 1998);
    /// Martinez, Rueda, and Feito describe boolean selection from classified
    /// segments (F. Martinez, A. J. Rueda, and F. R. Feito, "A new algorithm
    /// for computing Boolean operations on polygons," 2009). Cached boxes only
    /// prune decided misses, so boundary and overlap uncertainty is preserved.
    pub fn boolean_boundary_loops(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        crate::prepared_boolean::boolean_boundary_loops_between_prepared(self, other, op, policy)
    }

    /// Computes closed boolean boundary loops against an ordinary region view.
    ///
    /// This is a mixed prepared/unprepared convenience path: the left operand's
    /// cache is reused, the right operand is prepared for this call, and the
    /// prepared-prepared traversal described in
    /// [`PreparedRegionView2::boolean_boundary_loops`] remains authoritative.
    /// The transient right-side cache follows the same candidate-pruning role
    /// as Bentley and Ottmann's broad-phase intersection reporting setup, while
    /// the final boundary traversal still follows the Greiner-Hormann and
    /// Martinez-Rueda-Feito split/classify/assemble model cited above.
    pub fn boolean_boundary_loops_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_boundary_loops(&other, op, policy)
    }

    /// Computes checked boolean boundary contours against another prepared
    /// region.
    ///
    /// This extends [`PreparedRegionView2::boolean_boundary_loops`] through the
    /// same checked-contour conversion and regularized contact fast paths used
    /// by [`RegionView2::boolean_boundary_contours`]. The prepared parts remain
    /// candidate filters only: Foster, Hormann, and Popa's degenerate
    /// clipping cases still surface as explicit boundary handling rather than
    /// as tolerance-based inside/outside choices.
    pub fn boolean_boundary_contours(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        crate::prepared_boolean::boolean_boundary_contours_between_prepared(
            self, other, op, fill_rule, policy,
        )
    }

    /// Computes checked boolean boundary contours against an ordinary region
    /// view.
    ///
    /// This prepares the right operand only for the duration of the call and
    /// then uses [`PreparedRegionView2::boolean_boundary_contours`]. Keeping the
    /// wrapper explicit makes one-prepared/many-unprepared workloads ergonomic
    /// without weakening the degenerate clipping behavior described by Foster,
    /// Hormann, and Popa for boundary contacts.
    pub fn boolean_boundary_contours_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_boundary_contours(&other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against another prepared region.
    ///
    /// This is the prepared analogue of [`RegionView2::boolean_region`]. It
    /// reuses cached event and point-classification broad phases before
    /// returning to the ordinary contour-nesting pass for final material/hole
    /// assignment, preserving the Vatti-style fill-state semantics already used
    /// by the non-prepared region pipeline.
    pub fn boolean_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Region2>> {
        crate::prepared_boolean::boolean_region_between_prepared(self, other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against an ordinary region view.
    ///
    /// The right operand is prepared transiently, after which the same prepared
    /// boolean-region path assigns resolved contours to material and hole bins.
    /// The nesting step remains the Hormann-Agathos boundary-first point
    /// classification used by [`RegionView2::boolean_region`].
    pub fn boolean_region_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Region2>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_region(&other, op, fill_rule, policy)
    }
}

impl CurveString2 {
    /// Builds a prepared borrowed curve string for repeated topology queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedCurveStringView2<'_> {
        PreparedCurveStringView2::from_curve_string(self, policy)
    }
}

impl Contour2 {
    /// Builds a prepared borrowed contour for repeated topology queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedContourView2<'_> {
        PreparedContourView2::from_contour(self, policy)
    }
}

impl Region2 {
    /// Builds a prepared borrowed view for repeated point classification.
    pub fn prepare_point_classifier(&self, policy: &CurvePolicy) -> PreparedRegionView2<'_> {
        PreparedRegionView2::from_region(self, policy)
    }

    /// Builds a prepared borrowed view for repeated point and event queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedRegionView2<'_> {
        PreparedRegionView2::from_region(self, policy)
    }
}

impl<'a> RegionView2<'a> {
    /// Builds a prepared borrowed view for repeated point classification.
    pub fn prepare_point_classifier(&self, policy: &CurvePolicy) -> PreparedRegionView2<'a> {
        PreparedRegionView2::from_region_view(self, policy)
    }

    /// Builds a prepared borrowed view for repeated point and event queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedRegionView2<'a> {
        PreparedRegionView2::from_region_view(self, policy)
    }

    /// Collects normalized topology events against a prepared right operand.
    ///
    /// This preserves operand order for callers that have already prepared the
    /// second region. The left view is prepared transiently, then the prepared
    /// event collector uses cached broad-phase boxes before exact intersection
    /// normalization.
    pub fn intersect_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionIntersectionSet> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.intersect_prepared_region(other, policy)
    }

    /// Computes closed boolean boundary loops against a prepared right operand.
    ///
    /// Use this when the right operand is reused across many ordinary region
    /// views, especially for non-commutative operations such as difference. The
    /// transient left cache only prunes decided misses; Greiner-Hormann style
    /// boundary traversal and Martinez-Rueda-Feito fragment selection remain
    /// unchanged from [`RegionView2::boolean_boundary_loops`].
    pub fn boolean_boundary_loops_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_boundary_loops(other, op, policy)
    }

    /// Computes checked boolean boundary contours against a prepared right
    /// operand.
    ///
    /// The operation order is `self op other`; the prepared right operand is not
    /// swapped to the left. Degenerate shared-boundary cases keep the same
    /// explicit Foster-Hormann-Popa style uncertainty/regularization behavior
    /// as the ordinary checked-contour API.
    pub fn boolean_boundary_contours_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_boundary_contours(other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against a prepared right
    /// operand.
    ///
    /// The prepared path still returns to the ordinary nesting classifier for
    /// material/hole assignment, so Hormann-Agathos boundary-first point
    /// classification remains the final arbiter for resolved output contours.
    pub fn boolean_region_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Region2>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_region(other, op, fill_rule, policy)
    }
}

fn prepared_contours<'a>(
    contours: &[&'a Contour2],
    policy: &CurvePolicy,
) -> Vec<PreparedContourView2<'a>>
where
{
    contours
        .iter()
        .map(|contour| PreparedContourView2::from_contour(contour, policy))
        .collect()
}

fn decided_segment_boxes(segments: &[crate::Segment2], policy: &CurvePolicy) -> Vec<Option<Aabb2>> {
    segments
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect()
}

fn union_all_decided_boxes<'a, I>(boxes: I, policy: &CurvePolicy) -> Option<Aabb2>
where
    I: IntoIterator<Item = Option<&'a Aabb2>>,
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

fn accumulate_depth(
    depth: &mut i32,
    contours: &[PreparedContourView2<'_>],
    point: &Point2,
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

fn collect_prepared_role_pairs(
    pairs: &mut Vec<RegionContourIntersection>,
    first_contours: &[PreparedContourView2<'_>],
    first_role: RegionContourRole,
    second_contours: &[PreparedContourView2<'_>],
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
