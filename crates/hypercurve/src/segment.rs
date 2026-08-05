//! Line and circular-arc segment primitives.

use hyperreal::{Real, RealSign, ZeroKnowledge as ZeroStatus};
use std::{
    sync::Arc,
    sync::{Mutex, OnceLock},
};

use crate::classify::{
    LineSide, classify_oriented_line, compare_reals, in_closed_unit_interval, is_zero, real_sign,
};
use crate::policy::resolve_certified_operation;
use crate::{
    Classification, CurveContext, CurveError, CurveOutcome, CurveResult, ParamRange, Point2,
};
use std::cmp::Ordering;

/// A finite line segment.
#[derive(Clone, Debug)]
pub struct LineSeg2 {
    start: Point2,
    end: Point2,
    endpoints_decided_distinct: bool,
    support: OnceLock<Arc<LineSupport2>>,
    has_retained_support: bool,
    // Shared supports retain source orientation; this bit recovers the
    // fragment's directed tangent without subtracting its wide endpoints.
    support_direction_reversed: bool,
    offset_provenance: Option<Arc<LineOffsetProvenance2>>,
}

impl PartialEq for LineSeg2 {
    fn eq(&self, other: &Self) -> bool {
        self.start == other.start && self.end == other.end && self.support == other.support
    }
}

#[derive(Debug, PartialEq)]
pub(crate) struct LineSupport2 {
    start: Point2,
    end: Point2,
}

#[derive(Debug, PartialEq)]
struct LineOffsetProvenance2 {
    source: Arc<LineSupport2>,
    left_distance: Real,
}

fn line_support_cell(support: Option<Arc<LineSupport2>>) -> OnceLock<Arc<LineSupport2>> {
    let cell = OnceLock::new();
    if let Some(support) = support {
        let _ = cell.set(support);
    }
    cell
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum RetainedLineRelation2 {
    Coincident,
    ParallelDistinct,
    Uncertain,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ArcSweepPointLocation2 {
    Endpoint,
    Interior,
    Outside,
}

impl LineSeg2 {
    /// Constructs a line segment and rejects equal endpoints when provable.
    pub fn try_new(start: Point2, end: Point2) -> CurveResult<Self> {
        if start == end {
            return Err(CurveError::ZeroLengthLine);
        }
        let endpoints_decided_distinct = match start.distance_squared(&end).zero_status() {
            ZeroStatus::Zero => return Err(CurveError::ZeroLengthLine),
            ZeroStatus::NonZero => true,
            ZeroStatus::Unknown => false,
        };
        Ok(Self {
            start,
            end,
            endpoints_decided_distinct,
            support: OnceLock::new(),
            has_retained_support: false,
            support_direction_reversed: false,
            offset_provenance: None,
        })
    }

    /// Constructs a line segment without validating endpoint distinctness.
    pub fn new_unchecked(start: Point2, end: Point2) -> Self {
        Self {
            start,
            end,
            endpoints_decided_distinct: false,
            support: OnceLock::new(),
            has_retained_support: false,
            support_direction_reversed: false,
            offset_provenance: None,
        }
    }

    /// Returns the segment start point.
    pub const fn start(&self) -> &Point2 {
        &self.start
    }

    /// Returns the segment end point.
    pub const fn end(&self) -> &Point2 {
        &self.end
    }

    /// Returns `(end.x - start.x, end.y - start.y)`.
    pub fn delta(&self) -> (Real, Real) {
        self.end.delta_from(&self.start)
    }

    pub(crate) fn support_delta(&self) -> (Real, Real) {
        self.support.get().map_or_else(
            || self.delta(),
            |support| support.end.delta_from(&support.start),
        )
    }

    /// Returns a support vector oriented with this segment's traversal.
    pub(crate) fn directed_support_delta(&self) -> (Real, Real) {
        let (x, y) = self.support_delta();
        if self.support_direction_reversed {
            (-x, -y)
        } else {
            (x, y)
        }
    }

    pub(crate) const fn has_retained_support(&self) -> bool {
        self.has_retained_support
    }

    pub(crate) const fn endpoints_decided_distinct(&self) -> bool {
        self.endpoints_decided_distinct
    }

    pub(crate) fn support_start(&self) -> &Point2 {
        self.support
            .get()
            .map_or(&self.start, |support| &support.start)
    }

    pub(crate) fn fragment_between(&self, start: Point2, end: Point2) -> CurveResult<Self> {
        if start == end {
            return Err(CurveError::ZeroLengthLine);
        }
        let endpoints_decided_distinct = match start.distance_squared(&end).zero_status() {
            ZeroStatus::Zero => return Err(CurveError::ZeroLengthLine),
            ZeroStatus::NonZero => true,
            ZeroStatus::Unknown => false,
        };
        Ok(Self {
            start,
            end,
            endpoints_decided_distinct,
            support: line_support_cell(Some(self.fragment_support())),
            has_retained_support: true,
            support_direction_reversed: self.support_direction_reversed,
            offset_provenance: self.offset_provenance.clone(),
        })
    }

    pub(crate) fn fragment_between_after_distinct_endpoints(
        &self,
        start: Point2,
        end: Point2,
        support: Arc<LineSupport2>,
    ) -> Self {
        Self {
            start,
            end,
            endpoints_decided_distinct: true,
            support: line_support_cell(Some(support)),
            has_retained_support: true,
            support_direction_reversed: self.support_direction_reversed,
            offset_provenance: self.offset_provenance.clone(),
        }
    }

    pub(crate) fn fragment_support(&self) -> Arc<LineSupport2> {
        self.support
            .get_or_init(|| {
                Arc::new(LineSupport2 {
                    start: self.start.clone(),
                    end: self.end.clone(),
                })
            })
            .clone()
    }

    pub(crate) fn retained_support_intervals_decided_disjoint(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<bool> {
        if !self.has_retained_support || !other.has_retained_support {
            return None;
        }
        let first_support = self.support.get()?;
        let second_support = other.support.get()?;
        if !Arc::ptr_eq(first_support, second_support) {
            return None;
        }
        let use_x = match compare_reals(first_support.start.x(), first_support.end.x(), policy) {
            Some(Ordering::Less | Ordering::Greater) => true,
            Some(Ordering::Equal) => false,
            None => match compare_reals(first_support.start.y(), first_support.end.y(), policy) {
                Some(Ordering::Less | Ordering::Greater) => false,
                Some(Ordering::Equal) | None => return None,
            },
        };
        let (first_start, first_end) = ordered_line_endpoints(self, use_x, policy)?;
        let (second_start, second_end) = ordered_line_endpoints(other, use_x, policy)?;
        Some(
            compare_reals(first_end, second_start, policy) == Some(Ordering::Less)
                || compare_reals(second_end, first_start, policy) == Some(Ordering::Less),
        )
    }

    pub(crate) fn offset_between(
        &self,
        start: Point2,
        end: Point2,
        distance: Real,
    ) -> CurveResult<Self> {
        let mut offset = Self::try_new(start, end)?;
        let provenance = match self.offset_provenance.as_ref() {
            None => LineOffsetProvenance2 {
                source: self.fragment_support(),
                left_distance: distance,
            },
            Some(provenance) => LineOffsetProvenance2 {
                source: provenance.source.clone(),
                left_distance: &provenance.left_distance + &distance,
            },
        };
        offset.offset_provenance = Some(Arc::new(provenance));
        Ok(offset)
    }

    pub(crate) fn retained_offset_relation(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<RetainedLineRelation2> {
        let first = self.offset_provenance.as_ref()?;
        let second = other.offset_provenance.as_ref()?;
        let second_distance = if first.source == second.source {
            second.left_distance.clone()
        } else if first.source.start == second.source.end && first.source.end == second.source.start
        {
            -second.left_distance.clone()
        } else {
            return None;
        };

        Some(
            match compare_reals(&first.left_distance, &second_distance, policy) {
                Some(Ordering::Equal) => RetainedLineRelation2::Coincident,
                Some(Ordering::Less | Ordering::Greater) => RetainedLineRelation2::ParallelDistinct,
                None => RetainedLineRelation2::Uncertain,
            },
        )
    }

    /// Certifies that this retained offset fragment still traverses in the
    /// direction of its source support.
    ///
    /// Inward polygon wavefronts reverse source-parallel edges after local
    /// collapse. Splitting an offset at self contacts preserves per-line
    /// provenance, so this predicate lets the arrangement discard those
    /// post-collapse cycles without using a floating-point distance sample.
    pub(crate) fn retained_offset_direction_matches_source(
        &self,
        policy: &CurveContext,
    ) -> Classification<bool> {
        let Some(provenance) = self.offset_provenance.as_ref() else {
            return Classification::Uncertain(crate::UncertaintyReason::Unsupported);
        };
        let (offset_x, offset_y) = self.delta();
        let source_x = provenance.source.end.x() - provenance.source.start.x();
        let source_y = provenance.source.end.y() - provenance.source.start.y();
        let direction_dot = &offset_x * &source_x + &offset_y * &source_y;
        match real_sign(&direction_dot, policy) {
            Some(RealSign::Positive) => Classification::Decided(true),
            Some(RealSign::Zero | RealSign::Negative) => Classification::Decided(false),
            None => Classification::Uncertain(crate::UncertaintyReason::RealSign),
        }
    }

    pub(crate) fn map_points<F>(&self, mut map: F) -> CurveResult<Self>
    where
        F: FnMut(&Point2) -> Point2,
    {
        let start = map(&self.start);
        let end = map(&self.end);
        self.map_points_between(start, end, map)
    }

    pub(crate) fn map_points_between<F>(
        &self,
        start: Point2,
        end: Point2,
        mut map: F,
    ) -> CurveResult<Self>
    where
        F: FnMut(&Point2) -> Point2,
    {
        if start == end {
            return Err(CurveError::ZeroLengthLine);
        }
        let endpoints_decided_distinct = match start.distance_squared(&end).zero_status() {
            ZeroStatus::Zero => return Err(CurveError::ZeroLengthLine),
            ZeroStatus::NonZero => true,
            ZeroStatus::Unknown => false,
        };
        let support = self
            .has_retained_support
            .then(|| self.support.get())
            .flatten()
            .map(|support| {
                Arc::new(LineSupport2 {
                    start: map(&support.start),
                    end: map(&support.end),
                })
            });
        Ok(Self {
            start,
            end,
            endpoints_decided_distinct,
            support: line_support_cell(support),
            has_retained_support: self.has_retained_support,
            support_direction_reversed: self.support_direction_reversed,
            // An arbitrary point map need not preserve signed offset distance.
            offset_provenance: None,
        })
    }

    /// Returns squared segment length.
    pub fn length_squared(&self) -> Real {
        self.start.distance_squared(&self.end)
    }

    /// Returns the point at affine parameter `t`, where `0` is start and `1` is end.
    pub fn point_at(&self, t: Real) -> Point2 {
        let interpolated = self.start.lerp(&self.end, t);
        Point2::new(
            if self.start.x() == self.end.x() {
                self.start.x().clone()
            } else {
                interpolated.x().clone()
            },
            if self.start.y() == self.end.y() {
                self.start.y().clone()
            } else {
                interpolated.y().clone()
            },
        )
    }

    /// Returns this segment with traversal direction reversed.
    pub fn reversed(&self) -> Self {
        let offset_provenance = self.offset_provenance.as_ref().map(|provenance| {
            Arc::new(LineOffsetProvenance2 {
                source: Arc::new(LineSupport2 {
                    start: provenance.source.end.clone(),
                    end: provenance.source.start.clone(),
                }),
                left_distance: -provenance.left_distance.clone(),
            })
        });
        Self {
            start: self.end.clone(),
            end: self.start.clone(),
            endpoints_decided_distinct: self.endpoints_decided_distinct,
            support: line_support_cell(
                self.has_retained_support
                    .then(|| self.support.get().cloned())
                    .flatten(),
            ),
            has_retained_support: self.has_retained_support,
            support_direction_reversed: self.has_retained_support
                && !self.support_direction_reversed,
            offset_provenance,
        }
    }

    pub(crate) fn into_reversed(mut self) -> Self {
        if self.offset_provenance.is_some() {
            return self.reversed();
        }
        std::mem::swap(&mut self.start, &mut self.end);
        if self.has_retained_support {
            self.support_direction_reversed = !self.support_direction_reversed;
        } else {
            self.support.take();
        }
        self
    }

    /// Classifies a point relative to this oriented line segment's supporting line.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> Classification<LineSide> {
        let support_start = self.support_start();
        let support_end = self.support.get().map_or(&self.end, |support| &support.end);
        classify_oriented_line(support_start, support_end, point, policy)
    }

    /// Classifies whether a point lies on this finite line segment.
    pub fn contains_point(&self, point: &Point2, policy: &CurveContext) -> Classification<bool> {
        let side = match self.classify_point(point, policy) {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        self.contains_point_with_classified_side(point, side, policy)
    }

    pub(crate) fn contains_point_with_classified_side(
        &self,
        point: &Point2,
        side: LineSide,
        policy: &CurveContext,
    ) -> Classification<bool> {
        if side != LineSide::On {
            return Classification::Decided(false);
        }

        match parameter_on_line(self, point, policy) {
            ParameterOnLine::Decided(t) => in_closed_unit_interval(&t, policy)
                .map(Classification::Decided)
                .unwrap_or(Classification::Uncertain(
                    crate::UncertaintyReason::Ordering,
                )),
            ParameterOnLine::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Returns conservative structural facts for this line segment.
    ///
    /// Axis-aligned and shared-scale facts are scheduling hints only. They help
    /// select faster exact kernels without becoming a substitute for the
    /// orientation predicates used for topology.
    pub fn structural_facts(&self) -> crate::LineSeg2Facts {
        crate::facts::compute_line_segment_facts(self)
    }
}

/// A finite circular arc segment.
#[derive(Clone, Debug)]
pub struct CircularArc2 {
    // Geometry and lazy facts share one allocation. Arc clones already share
    // the immutable geometry, while native segment vectors no longer inherit
    // the full inline arc payload for every line element.
    pub(crate) retained_facts: Arc<CircularArcRetainedFacts2>,
}

#[derive(Debug)]
pub(crate) struct CircularArcRetainedFacts2 {
    start: Point2,
    end: Point2,
    center: Point2,
    radius_squared: Real,
    endpoints_on_stored_circle: bool,
    clockwise: bool,
    source_bulge: Option<Real>,
    structural_facts: OnceLock<Box<crate::CircularArc2Facts>>,
    pub(crate) sweep_kind: crate::policy::PolicyEvaluationCache<crate::arc_bezier::ArcSweepKind>,
    pub(crate) bezier_decomposition:
        crate::policy::PolicyEvaluationCache<crate::CircularArcBezierDecomposition2>,
    representative_point: crate::policy::PolicyEvaluationCache<Point2>,
    directed_sweep_angle: crate::policy::PolicyEvaluationCache<Real>,
    parameter_lineage: OnceLock<Box<CircularArcParameterLineage2>>,
    parameter_witnesses: OnceLock<Box<Mutex<Vec<CircularArcParameterWitness2>>>>,
    fragments: OnceLock<Box<Mutex<Vec<CircularArcFragmentWitness2>>>>,
}

impl CircularArcRetainedFacts2 {
    fn new(
        start: Point2,
        end: Point2,
        center: Point2,
        radius_squared: Real,
        endpoints_on_stored_circle: bool,
        clockwise: bool,
        source_bulge: Option<Real>,
    ) -> Self {
        Self {
            start,
            end,
            center,
            radius_squared,
            endpoints_on_stored_circle,
            clockwise,
            source_bulge,
            structural_facts: OnceLock::new(),
            sweep_kind: crate::policy::PolicyEvaluationCache::new(),
            bezier_decomposition: crate::policy::PolicyEvaluationCache::new(),
            representative_point: crate::policy::PolicyEvaluationCache::new(),
            directed_sweep_angle: crate::policy::PolicyEvaluationCache::new(),
            parameter_lineage: OnceLock::new(),
            parameter_witnesses: OnceLock::new(),
            fragments: OnceLock::new(),
        }
    }
}

#[derive(Clone, Debug)]
struct CircularArcParameterLineage2 {
    root_start: Point2,
    root_sweep_angle: Real,
    root_range: ParamRange,
}

#[derive(Clone, Debug)]
struct CircularArcParameterWitness2 {
    parameter: Real,
    point: Point2,
}

#[derive(Clone, Debug)]
struct CircularArcFragmentWitness2 {
    source_range: ParamRange,
    start: Point2,
    end: Point2,
    fragment: CircularArc2,
}

const MAX_RETAINED_ARC_FRAGMENTS: usize = 8;

impl PartialEq for CircularArc2 {
    fn eq(&self, other: &Self) -> bool {
        self.start() == other.start()
            && self.end() == other.end()
            && self.center() == other.center()
            && self.radius_squared_ref() == other.radius_squared_ref()
            && self.is_clockwise() == other.is_clockwise()
            && self.bulge() == other.bulge()
    }
}

impl CircularArc2 {
    fn data(&self) -> &CircularArcRetainedFacts2 {
        &self.retained_facts
    }

    fn from_geometry(
        start: Point2,
        end: Point2,
        center: Point2,
        radius_squared: Real,
        endpoints_on_stored_circle: bool,
        clockwise: bool,
        source_bulge: Option<Real>,
    ) -> Self {
        Self {
            retained_facts: Arc::new(CircularArcRetainedFacts2::new(
                start,
                end,
                center,
                radius_squared,
                endpoints_on_stored_circle,
                clockwise,
                source_bulge,
            )),
        }
    }

    /// Constructs a circular arc from endpoints, center, and orientation.
    pub fn try_from_center(
        start: Point2,
        end: Point2,
        center: Point2,
        clockwise: bool,
    ) -> CurveResult<Self> {
        let start_radius_squared = start.distance_squared(&center);
        if start_radius_squared.zero_status() == ZeroStatus::Zero {
            return Err(CurveError::ZeroRadiusArc);
        }

        let end_radius_squared = end.distance_squared(&center);
        let mismatch = &start_radius_squared - &end_radius_squared;
        let endpoints_on_stored_circle = match mismatch.zero_status() {
            ZeroStatus::Zero => true,
            ZeroStatus::NonZero => return Err(CurveError::RadiusMismatch),
            ZeroStatus::Unknown => false,
        };

        Ok(Self::from_geometry(
            start,
            end,
            center,
            start_radius_squared,
            endpoints_on_stored_circle,
            clockwise,
            None,
        ))
    }

    pub(crate) fn new_unchecked_with_radius(
        start: Point2,
        end: Point2,
        center: Point2,
        radius_squared: Real,
        clockwise: bool,
        bulge: Option<Real>,
    ) -> Self {
        Self::from_geometry(start, end, center, radius_squared, false, clockwise, bulge)
    }

    pub(crate) fn new_with_certified_radius(
        start: Point2,
        end: Point2,
        center: Point2,
        radius_squared: Real,
        clockwise: bool,
        bulge: Option<Real>,
    ) -> Self {
        Self::from_geometry(start, end, center, radius_squared, true, clockwise, bulge)
    }

    pub(crate) fn new_with_certified_radius_and_sweep(
        start: Point2,
        end: Point2,
        center: Point2,
        radius_squared: Real,
        clockwise: bool,
        sweep_kind: crate::arc_bezier::ArcSweepKind,
    ) -> Self {
        let arc =
            Self::new_with_certified_radius(start, end, center, radius_squared, clockwise, None);
        arc.retained_facts.sweep_kind.seed_certified(sweep_kind);
        arc
    }

    pub(crate) fn try_from_center_with_bulge(
        start: Point2,
        end: Point2,
        center: Point2,
        clockwise: bool,
        bulge: Option<Real>,
    ) -> CurveResult<Self> {
        let mut arc = Self::try_from_center(start, end, center, clockwise)?;
        Arc::get_mut(&mut arc.retained_facts)
            .expect("new arc retained facts are uniquely owned")
            .source_bulge = bulge;
        Ok(arc)
    }

    /// Constructs a circular arc from CAD bulge geometry.
    ///
    /// The formula keeps the center computation in rational operations:
    /// `center = midpoint + left_perp(chord) * ((1 - b^2) / (4b))`.
    pub fn from_bulge(start: Point2, end: Point2, bulge: Real) -> CurveResult<Self> {
        if start.distance_squared(&end).zero_status() == ZeroStatus::Zero {
            return Err(CurveError::ZeroLengthLine);
        }

        let clockwise = clockwise_from_bulge(&bulge)?;
        let four_b = Real::from(4_i8) * &bulge;
        let b2 = &bulge * &bulge;
        let offset_factor = ((Real::one() - &b2) / four_b)?;
        let two = Real::from(2_i8);
        let mid_x = ((start.x() + end.x()) / &two)?;
        let mid_y = ((start.y() + end.y()) / &two)?;
        let (dx, dy) = end.delta_from(&start);

        let center = Point2::new(
            mid_x - (&dy * &offset_factor),
            mid_y + (&dx * &offset_factor),
        );

        let mut arc = Self::try_from_center(start, end, center, clockwise)?;
        Arc::get_mut(&mut arc.retained_facts)
            .expect("new arc retained facts are uniquely owned")
            .source_bulge = Some(bulge);
        Ok(arc)
    }

    /// Returns the arc start point.
    pub fn start(&self) -> &Point2 {
        &self.data().start
    }

    /// Returns the arc end point.
    pub fn end(&self) -> &Point2 {
        &self.data().end
    }

    /// Returns the arc center.
    pub fn center(&self) -> &Point2 {
        &self.data().center
    }

    /// Returns the squared radius.
    pub fn radius_squared(&self) -> Real {
        self.retained_facts.radius_squared.clone()
    }

    /// Returns the stored squared radius by reference.
    pub fn radius_squared_ref(&self) -> &Real {
        &self.data().radius_squared
    }

    pub(crate) fn endpoints_on_stored_circle_are_certified(&self) -> bool {
        self.data().endpoints_on_stored_circle
    }

    /// Returns whether this arc travels clockwise from start to end.
    pub fn is_clockwise(&self) -> bool {
        self.data().clockwise
    }

    /// Returns the source bulge when this arc was constructed from one.
    pub fn bulge(&self) -> Option<&Real> {
        self.retained_facts.source_bulge.as_ref()
    }

    /// Classifies whether a point lies inside this arc's angular sweep.
    ///
    /// Supports minor, semicircular, major, and full-circle sweeps. The point
    /// does not have to be on the circle; callers that need point-on-arc
    /// semantics should also compare squared distance to
    /// [`CircularArc2::radius_squared`].
    /// The half-plane tests are the finite-arc containment counterpart to the
    /// circle and arc primitive tests catalogued by standard geometric constructions.
    pub fn contains_sweep_point(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        if point_matches_arc_endpoint(self, point, policy) == Some(true) {
            return Classification::Decided(true);
        }

        self.contains_non_endpoint_sweep_point(point, policy)
    }

    fn contains_non_endpoint_sweep_point(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        let sweep_kind = match crate::arc_bezier::classify_sweep_with_policy(self, policy) {
            Ok(Classification::Decided(kind)) => kind,
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(crate::UncertaintyReason::Predicate),
        };
        if sweep_kind == crate::arc_bezier::ArcSweepKind::FullCircle {
            return Classification::Decided(true);
        }

        let start_side = classify_oriented_line(self.center(), self.start(), point, policy);
        let end_side = classify_oriented_line(self.center(), self.end(), point, policy);
        let (Classification::Decided(start_side), Classification::Decided(end_side)) =
            (start_side, end_side)
        else {
            return Classification::Uncertain(crate::UncertaintyReason::Predicate);
        };

        self.contains_classified_sweep_sides(start_side, end_side, sweep_kind)
    }

    pub(crate) fn strict_sweep_point_location(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> Classification<ArcSweepPointLocation2> {
        match point_matches_arc_endpoint(self, point, policy) {
            Some(true) => {
                return Classification::Decided(ArcSweepPointLocation2::Endpoint);
            }
            Some(false) => {}
            None => {
                return Classification::Uncertain(crate::UncertaintyReason::RealSign);
            }
        }
        self.contains_non_endpoint_sweep_point(point, policy)
            .map(|contains| {
                if contains {
                    ArcSweepPointLocation2::Interior
                } else {
                    ArcSweepPointLocation2::Outside
                }
            })
    }

    pub(crate) fn contains_classified_sweep_sides(
        &self,
        start_side: LineSide,
        end_side: LineSide,
        sweep_kind: crate::arc_bezier::ArcSweepKind,
    ) -> Classification<bool> {
        let start_contains = if self.is_clockwise() {
            matches!(start_side, LineSide::Right | LineSide::On)
        } else {
            matches!(start_side, LineSide::Left | LineSide::On)
        };
        let end_contains = if self.is_clockwise() {
            matches!(end_side, LineSide::Left | LineSide::On)
        } else {
            matches!(end_side, LineSide::Right | LineSide::On)
        };
        Classification::Decided(if sweep_kind == crate::arc_bezier::ArcSweepKind::Major {
            start_contains || end_contains
        } else {
            start_contains && end_contains
        })
    }

    /// Classifies whether a point lies on this finite circular arc.
    pub fn contains_point(&self, point: &Point2, policy: &CurveContext) -> Classification<bool> {
        if point_matches_arc_endpoint(self, point, policy) == Some(true) {
            return Classification::Decided(true);
        }
        let radius_delta = point.distance_squared(self.center()) - self.radius_squared();
        match is_zero(&radius_delta, policy) {
            Some(false) => Classification::Decided(false),
            Some(true) => self.contains_sweep_point(point, policy),
            None => Classification::Uncertain(crate::UncertaintyReason::RealSign),
        }
    }

    /// Returns conservative structural facts for this arc.
    ///
    /// These facts can schedule future circle/arc exact kernels while leaving
    /// topological decisions to certified predicates and exact sign queries.
    pub fn structural_facts(&self) -> crate::CircularArc2Facts {
        **self
            .retained_facts
            .structural_facts
            .get_or_init(|| Box::new(crate::facts::compute_circular_arc_facts(self)))
    }

    /// Returns a point in the interior of this arc's supported sweep.
    ///
    /// The point is the exact midpoint of the requested minor, semicircular,
    /// major, or full-circle traversal. Arc fragments retain their source
    /// angular parameterization, so repeated and nested trims evaluate this
    /// point without rebuilding nested trigonometric rotations.
    pub fn representative_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        crate::policy::resolve_cached_evaluation(
            &self.retained_facts.representative_point,
            policy,
            |attempt| self.compute_representative_point(attempt),
        )
        .map(|classification| classification.map(Clone::clone))
    }

    fn compute_representative_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        let half = (Real::one() / Real::from(2_i8))?;
        if self.retained_facts.parameter_lineage.get().is_some() {
            return self.point_at_sweep_fraction(&half, policy);
        }
        match self.rational_bezier_decomposition_with_policy(policy) {
            Ok(Classification::Decided(decomposition)) => {
                match decomposition.point_at_with_policy(&half, policy) {
                    Ok(point) => Ok(Classification::Decided(point)),
                    Err(crate::ExactCurveError::Invalid { cause, .. }) => Err(cause),
                    Err(crate::ExactCurveError::Blocked(blocker)) => {
                        Ok(Classification::Uncertain(blocker.reason()))
                    }
                }
            }
            Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(reason)),
            Err(crate::ExactCurveError::Invalid { cause, .. }) => Err(cause),
            Err(crate::ExactCurveError::Blocked(blocker)) => {
                Ok(Classification::Uncertain(blocker.reason()))
            }
        }
    }

    /// Returns the exact directed-angular sweep fraction of a point on this arc.
    ///
    /// Zero is the arc start and one is the arc end. Interior values increase
    /// in traversal order for clockwise, counterclockwise, minor, major, and
    /// full-circle arcs. This is an angular ordering parameter; it is not the
    /// piecewise rational-Bezier evaluation parameter returned by
    /// [`CircularArc2::rational_bezier_decomposition`].
    pub fn sweep_fraction(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        match self.contains_point(point, policy) {
            Classification::Decided(true) => self.sweep_fraction_for_incident_point(point, policy),
            Classification::Decided(false) => Ok(Classification::Uncertain(
                crate::UncertaintyReason::Boundary,
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Evaluates this arc at a directed-angular sweep fraction.
    ///
    /// Zero returns the stored start point and one returns the stored end point.
    /// Interior fractions follow traversal order for clockwise,
    /// counterclockwise, minor, major, and full-circle arcs. This is the
    /// inverse parameterization of [`CircularArc2::sweep_fraction`], not the
    /// piecewise rational-Bezier parameterization used by
    /// [`CircularArc2::rational_bezier_decomposition`].
    pub fn point_at_sweep_fraction(
        &self,
        fraction: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        match in_closed_unit_interval(fraction, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidCurveParameter),
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
        match compare_reals(fraction, &Real::zero(), policy) {
            Some(Ordering::Equal) => return Ok(Classification::Decided(self.start().clone())),
            Some(_) => {}
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
        match compare_reals(fraction, &Real::one(), policy) {
            Some(Ordering::Equal) => return Ok(Classification::Decided(self.end().clone())),
            Some(_) => {}
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
        if let Some(point) = self.retained_parameter_witness(fraction) {
            return Ok(Classification::Decided(point));
        }

        let sweep_angle = match self.retained_directed_sweep_angle(policy)? {
            Classification::Decided(angle) => angle,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (radial, traversal_angle) =
            if let Some(lineage) = self.retained_facts.parameter_lineage.get() {
                let root_width = lineage.root_range.end() - lineage.root_range.start();
                let root_fraction = lineage.root_range.start() + &(root_width * fraction);
                (
                    lineage.root_start.delta_from(self.center()),
                    &lineage.root_sweep_angle * root_fraction,
                )
            } else {
                (
                    self.start().delta_from(self.center()),
                    sweep_angle * fraction,
                )
            };
        let signed_angle = if self.is_clockwise() {
            -traversal_angle
        } else {
            traversal_angle
        };
        let cosine = signed_angle.clone().cos();
        let sine = signed_angle.sin();
        Ok(Classification::Decided(Point2::new(
            self.center().x() + (&radial.0 * &cosine) - (&radial.1 * &sine),
            self.center().y() + (&radial.0 * sine) + (&radial.1 * cosine),
        )))
    }

    /// Splits this arc at a strict interior directed-angular sweep fraction.
    ///
    /// Unlike splitting through the arc's rational-Bézier public parameter,
    /// this operation retains the original angular lineage. Nested splits
    /// therefore evaluate their endpoints from the root circle and do not
    /// accumulate algebraically equivalent rational-projection expressions.
    pub fn split_at_sweep_fraction(
        &self,
        fraction: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Self, Self)>> {
        let middle = match self.point_at_sweep_fraction(fraction, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.split_at_retained_sweep_point(fraction, middle, policy)
    }

    /// Splits this arc through an exact point with a retained sweep fraction.
    ///
    /// `point` must be retained exact contact evidence already certified on
    /// this arc at `fraction`. This method validates the strict fraction but
    /// deliberately does not re-solve point incidence: doing so would discard
    /// the caller's contact certificate and reconstruct the same image through
    /// trigonometric predicates. The retained point becomes the shared
    /// endpoint while the source arc's angular lineage is preserved.
    pub fn split_at_retained_sweep_point(
        &self,
        fraction: &Real,
        point: Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Self, Self)>> {
        match compare_reals(fraction, &Real::zero(), policy) {
            Some(Ordering::Greater) => {}
            Some(_) => return Err(CurveError::InvalidCurveParameter),
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
        match compare_reals(fraction, &Real::one(), policy) {
            Some(Ordering::Less) => {}
            Some(_) => return Err(CurveError::InvalidCurveParameter),
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
        let first_range = ParamRange::new(Real::zero(), fraction.clone());
        let second_range = ParamRange::new(fraction.clone(), Real::one());
        let first = match self.fragment_between_sweep_range(
            self.start().clone(),
            point.clone(),
            &first_range,
            policy,
        )? {
            Classification::Decided(fragment) => fragment,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match self.fragment_between_sweep_range(
            point,
            self.end().clone(),
            &second_range,
            policy,
        )? {
            Classification::Decided(fragment) => fragment,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided((first, second)))
    }

    /// Returns the exact piecewise-rational public parameter at a directed
    /// angular sweep fraction.
    ///
    /// This is the parameter-space inverse needed when an angularly
    /// parameterized owner splits a native circular arc. The angular point is
    /// constructed exactly, then replayed through the retained rational
    /// quadratic decomposition's certified point-parameter solvers.
    pub fn parameter_at_sweep_fraction(
        &self,
        fraction: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        match compare_reals(fraction, &Real::zero(), policy) {
            Some(Ordering::Equal) => return Ok(Classification::Decided(Real::zero())),
            Some(Ordering::Greater) => {}
            Some(Ordering::Less) => return Err(CurveError::InvalidCurveParameter),
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
        match compare_reals(fraction, &Real::one(), policy) {
            Some(Ordering::Equal) => return Ok(Classification::Decided(Real::one())),
            Some(Ordering::Less) => {}
            Some(Ordering::Greater) => return Err(CurveError::InvalidCurveParameter),
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
        let decomposition = match self.rational_bezier_decomposition_with_policy(policy) {
            Ok(Classification::Decided(decomposition)) => decomposition,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(crate::ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            Err(crate::ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
        };
        let point = match self.point_at_sweep_fraction(fraction, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for span in decomposition.spans() {
            let (start, end) = span.parameter_range();
            match (
                compare_reals(start, fraction, policy),
                compare_reals(fraction, end, policy),
            ) {
                (
                    Some(Ordering::Less | Ordering::Equal),
                    Some(Ordering::Less | Ordering::Equal),
                ) => {
                    let width = end - start;
                    let start_radial = span.curve().start().delta_from(self.center());
                    let end_radial = span.curve().end().delta_from(self.center());
                    let point_radial = point.delta_from(self.center());
                    let radius_squared =
                        &start_radial.0 * &start_radial.0 + &start_radial.1 * &start_radial.1;
                    let start_point_dot =
                        &start_radial.0 * &point_radial.0 + &start_radial.1 * &point_radial.1;
                    let start_point_cross =
                        &start_radial.0 * &point_radial.1 - &start_radial.1 * &point_radial.0;
                    let start_end_dot =
                        &start_radial.0 * &end_radial.0 + &start_radial.1 * &end_radial.1;
                    let start_end_cross =
                        &start_radial.0 * &end_radial.1 - &start_radial.1 * &end_radial.0;
                    let local_rational = (start_point_cross * (&radius_squared + start_end_dot)
                        / ((&radius_squared + start_point_dot) * start_end_cross))?;
                    return Ok(Classification::Decided(start + &(width * local_rational)));
                }
                (Some(_), Some(_)) => {}
                _ => {
                    return Ok(Classification::Uncertain(
                        crate::UncertaintyReason::Ordering,
                    ));
                }
            }
        }
        Ok(Classification::Uncertain(
            crate::UncertaintyReason::Boundary,
        ))
    }

    /// Returns the exact positive angular sweep in traversal order.
    ///
    /// Counterclockwise and clockwise arcs both report a positive magnitude;
    /// orientation remains available through [`CircularArc2::is_clockwise`].
    /// Full circles report `tau`. The result is retained with the arc and is
    /// the angular measure used by [`CircularArc2::sweep_fraction`]. The
    /// returned [`CurveOutcome`] records whether exact angle classification
    /// consumed the `APPROXIMATE_512` terminal.
    #[inline(always)]
    pub fn directed_sweep_angle(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<&Real>>> {
        resolve_certified_operation(policy, |attempt| {
            self.retained_directed_sweep_angle(attempt)
        })
    }

    pub(crate) fn directed_sweep_angle_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        self.retained_directed_sweep_angle(policy)
            .map(|classification| classification.map(Clone::clone))
    }

    #[inline]
    fn retained_directed_sweep_angle(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<&Real>> {
        crate::policy::resolve_cached_evaluation(
            &self.retained_facts.directed_sweep_angle,
            policy,
            |attempt| self.compute_directed_sweep_angle(attempt),
        )
    }

    fn retained_parameter_witness(&self, parameter: &Real) -> Option<Point2> {
        let witnesses = self.retained_facts.parameter_witnesses.get()?;
        witnesses
            .lock()
            .expect("arc parameter witness cache mutex poisoned")
            .iter()
            .find(|witness| witness.parameter == *parameter)
            .map(|witness| witness.point.clone())
    }

    fn retain_parameter_witness(&self, parameter: &Real, point: &Point2) {
        let witnesses = self
            .retained_facts
            .parameter_witnesses
            .get_or_init(|| Box::new(Mutex::new(Vec::new())));
        let mut witnesses = witnesses
            .lock()
            .expect("arc parameter witness cache mutex poisoned");
        if witnesses
            .iter()
            .any(|witness| witness.parameter == *parameter)
        {
            return;
        }
        witnesses.push(CircularArcParameterWitness2 {
            parameter: parameter.clone(),
            point: point.clone(),
        });
    }

    fn compute_directed_sweep_angle(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        let sweep_kind = match crate::arc_bezier::classify_sweep_with_policy(self, policy) {
            Ok(Classification::Decided(kind)) => kind,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(crate::ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            Err(crate::ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
        };
        if sweep_kind == crate::arc_bezier::ArcSweepKind::FullCircle {
            return Ok(Classification::Decided(Real::tau()));
        }
        directed_radial_angle(self, self.end(), policy)
    }

    pub(crate) fn fragment_between_sweep_range(
        &self,
        start: Point2,
        end: Point2,
        source_range: &ParamRange,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if let Some(fragment) = self.retained_fragment(source_range, &start, &end) {
            return Ok(Classification::Decided(fragment));
        }
        let source_sweep = match self.retained_directed_sweep_angle(policy)? {
            Classification::Decided(angle) => angle.clone(),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source_sweep_is_certified = self
            .retained_facts
            .directed_sweep_angle
            .certified()
            .is_some();
        let (root_start, root_sweep_angle, parent_root_range) =
            if let Some(lineage) = self.retained_facts.parameter_lineage.get() {
                (
                    lineage.root_start.clone(),
                    lineage.root_sweep_angle.clone(),
                    lineage.root_range.clone(),
                )
            } else {
                (
                    self.start().clone(),
                    source_sweep,
                    ParamRange::new(Real::zero(), Real::one()),
                )
            };
        let parent_root_width = parent_root_range.end() - parent_root_range.start();
        let root_range = ParamRange::new(
            parent_root_range.start() + &(&parent_root_width * source_range.start()),
            parent_root_range.start() + &(&parent_root_width * source_range.end()),
        );
        let fragment_sweep = &root_sweep_angle * (root_range.end() - root_range.start());
        let fragment = Self::new_with_certified_radius(
            start.clone(),
            end.clone(),
            self.center().clone(),
            self.radius_squared(),
            self.is_clockwise(),
            None,
        );
        let _ =
            fragment
                .retained_facts
                .parameter_lineage
                .set(Box::new(CircularArcParameterLineage2 {
                    root_start,
                    root_sweep_angle,
                    root_range,
                }));
        if source_sweep_is_certified {
            fragment
                .retained_facts
                .directed_sweep_angle
                .seed_certified(fragment_sweep.clone());
            if let Some(kind) =
                sweep_kind_from_directed_angle(&fragment_sweep, &policy.strict_counterpart())
            {
                fragment.retained_facts.sweep_kind.seed_certified(kind);
            }
        }
        self.retain_fragment(source_range, &start, &end, &fragment);
        Ok(Classification::Decided(fragment))
    }

    fn retained_fragment(
        &self,
        source_range: &ParamRange,
        start: &Point2,
        end: &Point2,
    ) -> Option<Self> {
        let fragments = self.retained_facts.fragments.get()?;
        fragments
            .lock()
            .expect("arc fragment cache mutex poisoned")
            .iter()
            .find(|witness| {
                witness.source_range == *source_range
                    && witness.start == *start
                    && witness.end == *end
            })
            .map(|witness| witness.fragment.clone())
    }

    fn retain_fragment(
        &self,
        source_range: &ParamRange,
        start: &Point2,
        end: &Point2,
        fragment: &Self,
    ) {
        let fragments = self
            .retained_facts
            .fragments
            .get_or_init(|| Box::new(Mutex::new(Vec::new())));
        let mut fragments = fragments.lock().expect("arc fragment cache mutex poisoned");
        if fragments.len() == MAX_RETAINED_ARC_FRAGMENTS {
            fragments.remove(0);
        }
        fragments.push(CircularArcFragmentWitness2 {
            source_range: source_range.clone(),
            start: start.clone(),
            end: end.clone(),
            fragment: fragment.clone(),
        });
    }

    pub(crate) fn sweep_fraction_for_incident_point(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        match points_equal(point, self.start(), policy) {
            Some(true) => return Ok(Classification::Decided(Real::zero())),
            Some(false) => {}
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::RealSign,
                ));
            }
        }
        match points_equal(point, self.end(), policy) {
            Some(true) => return Ok(Classification::Decided(Real::one())),
            Some(false) => {}
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::RealSign,
                ));
            }
        }

        let point_angle = match directed_radial_angle(self, point, policy)? {
            Classification::Decided(angle) => angle,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let total_angle = match self.retained_directed_sweep_angle(policy)? {
            Classification::Decided(angle) => angle.clone(),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let parameter = (point_angle / total_angle).map_err(CurveError::from)?;
        self.retain_parameter_witness(&parameter, point);
        Ok(Classification::Decided(parameter))
    }

    /// Returns this arc with traversal direction reversed.
    pub fn reversed(&self) -> Self {
        Self::from_geometry(
            self.end().clone(),
            self.start().clone(),
            self.center().clone(),
            self.radius_squared(),
            self.endpoints_on_stored_circle_are_certified(),
            !self.is_clockwise(),
            self.bulge().map(|bulge| -bulge.clone()),
        )
    }

    pub(crate) fn into_reversed(self) -> Self {
        let retained_facts = match Arc::try_unwrap(self.retained_facts) {
            Ok(retained_facts) => {
                return Self::from_geometry(
                    retained_facts.end,
                    retained_facts.start,
                    retained_facts.center,
                    retained_facts.radius_squared,
                    retained_facts.endpoints_on_stored_circle,
                    !retained_facts.clockwise,
                    retained_facts.source_bulge.map(|bulge| -bulge),
                );
            }
            Err(retained_facts) => retained_facts,
        };
        Self { retained_facts }.reversed()
    }
}

/// A native line or circular-arc segment.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum Segment2 {
    /// Straight line segment.
    Line(LineSeg2),
    /// Circular arc segment.
    Arc(CircularArc2),
}

impl Segment2 {
    /// Returns the segment primitive family without computing structural facts.
    pub const fn kind(&self) -> crate::SegmentKind {
        match self {
            Self::Line(_) => crate::SegmentKind::Line,
            Self::Arc(_) => crate::SegmentKind::Arc,
        }
    }

    /// Constructs a native segment from a bulge value.
    ///
    /// Zero bulge maps to a line. Nonzero bulge maps to a circular arc.
    pub fn from_bulge(start: Point2, end: Point2, bulge: Real) -> CurveResult<Self> {
        match bulge.zero_status() {
            ZeroStatus::Zero => LineSeg2::try_new(start, end).map(Self::Line),
            ZeroStatus::NonZero => CircularArc2::from_bulge(start, end, bulge).map(Self::Arc),
            ZeroStatus::Unknown => Err(CurveError::AmbiguousBulge),
        }
    }

    /// Returns the segment start point.
    pub fn start(&self) -> &Point2 {
        match self {
            Self::Line(line) => line.start(),
            Self::Arc(arc) => arc.start(),
        }
    }

    /// Returns the segment end point.
    pub fn end(&self) -> &Point2 {
        match self {
            Self::Line(line) => line.end(),
            Self::Arc(arc) => arc.end(),
        }
    }

    /// Classifies whether a point lies on this finite segment.
    pub fn contains_point(&self, point: &Point2, policy: &CurveContext) -> Classification<bool> {
        match self {
            Self::Line(line) => line.contains_point(point, policy),
            Self::Arc(arc) => arc.contains_point(point, policy),
        }
    }

    /// Returns conservative structural facts for this native segment.
    pub fn structural_facts(&self) -> crate::Segment2Facts {
        crate::facts::segment_facts(self)
    }

    /// Returns a point in the interior of this segment.
    pub fn representative_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        let half = (Real::one() / Real::from(2_i8))?;
        self.point_at(&half, policy)
    }

    /// Evaluates this segment at a normalized traversal parameter in `[0, 1]`.
    pub fn point_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidCurveParameter),
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
        match self {
            Self::Line(line) => Ok(Classification::Decided(line.point_at(parameter.clone()))),
            Self::Arc(arc) => match arc.rational_bezier_decomposition_with_policy(policy) {
                Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(reason)),
                Ok(Classification::Decided(decomposition)) => {
                    match decomposition.point_at_with_policy(parameter, policy) {
                        Ok(point) => Ok(Classification::Decided(point)),
                        Err(crate::ExactCurveError::Invalid { cause, .. }) => Err(cause),
                        Err(crate::ExactCurveError::Blocked(blocker)) => {
                            Ok(Classification::Uncertain(blocker.reason()))
                        }
                    }
                }
                Err(crate::ExactCurveError::Invalid { cause, .. }) => Err(cause),
                Err(crate::ExactCurveError::Blocked(blocker)) => {
                    Ok(Classification::Uncertain(blocker.reason()))
                }
            },
        }
    }

    /// Returns this segment with traversal direction reversed.
    pub fn reversed(&self) -> Self {
        match self {
            Self::Line(line) => Self::Line(line.reversed()),
            Self::Arc(arc) => Self::Arc(arc.reversed()),
        }
    }

    pub(crate) fn into_reversed(self) -> Self {
        match self {
            Self::Line(line) => Self::Line(line.into_reversed()),
            Self::Arc(arc) => Self::Arc(arc.into_reversed()),
        }
    }
}

#[allow(clippy::large_enum_variant)]
enum ParameterOnLine {
    Decided(Real),
    Uncertain(crate::UncertaintyReason),
}

fn parameter_on_line(line: &LineSeg2, point: &Point2, policy: &CurveContext) -> ParameterOnLine {
    let (dx, dy) = line.delta();
    let delta = point.delta_from(line.start());

    match is_zero(&dx, policy) {
        Some(false) => match delta.0 / dx {
            Ok(t) => ParameterOnLine::Decided(t),
            Err(_) => ParameterOnLine::Uncertain(crate::UncertaintyReason::RealSign),
        },
        Some(true) => match delta.1 / dy {
            Ok(t) => ParameterOnLine::Decided(t),
            Err(_) => ParameterOnLine::Uncertain(crate::UncertaintyReason::RealSign),
        },
        None => match is_zero(&dy, policy) {
            Some(false) => match delta.1 / dy {
                Ok(t) => ParameterOnLine::Decided(t),
                Err(_) => ParameterOnLine::Uncertain(crate::UncertaintyReason::RealSign),
            },
            Some(true) => ParameterOnLine::Uncertain(crate::UncertaintyReason::RealSign),
            None => ParameterOnLine::Uncertain(crate::UncertaintyReason::RealSign),
        },
    }
}

fn clockwise_from_bulge(bulge: &Real) -> CurveResult<bool> {
    if let Some(sign) = bulge.structural_facts().sign {
        return match sign {
            RealSign::Negative => Ok(true),
            RealSign::Positive => Ok(false),
            RealSign::Zero => Err(CurveError::AmbiguousBulge),
        };
    }

    // Bulge sign chooses the arc sweep orientation, so route it through the
    // shared predicate policy used by the rest of curve topology.
    match crate::classify::real_sign(bulge, &CurveContext::STRICT) {
        Some(RealSign::Negative) => Ok(true),
        Some(RealSign::Positive) => Ok(false),
        Some(RealSign::Zero) => Err(CurveError::AmbiguousBulge),
        None => Err(CurveError::AmbiguousBulge),
    }
}

fn point_matches_arc_endpoint(
    arc: &CircularArc2,
    point: &Point2,
    policy: &CurveContext,
) -> Option<bool> {
    let start_distance = point.distance_squared(arc.start());
    if crate::classify::is_zero(&start_distance, policy)? {
        return Some(true);
    }
    let end_distance = point.distance_squared(arc.end());
    crate::classify::is_zero(&end_distance, policy)
}

fn ordered_line_endpoints<'a>(
    line: &'a LineSeg2,
    use_x: bool,
    policy: &CurveContext,
) -> Option<(&'a Real, &'a Real)> {
    let (start, end) = if use_x {
        (line.start().x(), line.end().x())
    } else {
        (line.start().y(), line.end().y())
    };
    match compare_reals(start, end, policy)? {
        Ordering::Less | Ordering::Equal => Some((start, end)),
        Ordering::Greater => Some((end, start)),
    }
}

fn points_equal(left: &Point2, right: &Point2, policy: &CurveContext) -> Option<bool> {
    if left == right {
        return Some(true);
    }
    crate::classify::is_zero(&left.distance_squared(right), policy)
}

fn directed_radial_angle(
    arc: &CircularArc2,
    point: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<Real>> {
    let start = arc.start().delta_from(arc.center());
    let radial = point.delta_from(arc.center());
    let cross = (&start.0 * &radial.1) - (&start.1 * &radial.0);
    let directed_cross = if arc.is_clockwise() { -cross } else { cross };
    let dot = (&start.0 * &radial.0) + (&start.1 * &radial.1);
    let Some(cross_sign) = crate::classify::real_sign(&directed_cross, policy) else {
        return Ok(Classification::Uncertain(
            crate::UncertaintyReason::RealSign,
        ));
    };
    let Some(dot_sign) = crate::classify::real_sign(&dot, policy) else {
        return Ok(Classification::Uncertain(
            crate::UncertaintyReason::RealSign,
        ));
    };
    let angle = match (cross_sign, dot_sign) {
        (RealSign::Zero, RealSign::Positive) => Real::zero(),
        (RealSign::Zero, RealSign::Negative) => Real::pi(),
        (RealSign::Zero, RealSign::Zero) => {
            return Err(CurveError::InvalidCurveParameter);
        }
        (RealSign::Positive, RealSign::Zero) => (Real::pi() / Real::from(2_i8))?,
        (RealSign::Negative, RealSign::Zero) => (Real::from(3_i8) * Real::pi() / Real::from(2_i8))?,
        (cross_sign, RealSign::Positive) => {
            let base = (directed_cross / dot)?.atan()?;
            if cross_sign == RealSign::Positive {
                base
            } else {
                base + Real::tau()
            }
        }
        (_, RealSign::Negative) => (directed_cross / dot)?.atan()? + Real::pi(),
    };
    Ok(Classification::Decided(angle))
}

fn sweep_kind_from_directed_angle(
    angle: &Real,
    policy: &CurveContext,
) -> Option<crate::arc_bezier::ArcSweepKind> {
    match compare_reals(angle, &Real::tau(), policy)? {
        Ordering::Equal => return Some(crate::arc_bezier::ArcSweepKind::FullCircle),
        Ordering::Greater => return None,
        Ordering::Less => {}
    }
    Some(match compare_reals(angle, &Real::pi(), policy)? {
        Ordering::Less => crate::arc_bezier::ArcSweepKind::Minor,
        Ordering::Equal => crate::arc_bezier::ArcSweepKind::Semicircle,
        Ordering::Greater => crate::arc_bezier::ArcSweepKind::Major,
    })
}

#[cfg(all(test, feature = "predicates"))]
mod policy_cache_tests {
    use super::*;

    fn point(x: i8, y: i8) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn approximate_sweep_fragment_does_not_gain_certified_cache_facts() {
        let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
        let center = Point2::new(Real::from(3_i8) + undecidable_zero, Real::one());
        let start = point(3, 0);
        let end = point(3, 2);
        let arc = CircularArc2::new_with_certified_radius(
            start.clone(),
            end,
            center.clone(),
            start.distance_squared(&center),
            false,
            None,
        );
        let half = (Real::one() / Real::from(2_i8)).unwrap();

        let Classification::Decided((first, _)) = arc
            .split_at_sweep_fraction(&half, &CurveContext::APPROXIMATE_512)
            .unwrap()
        else {
            panic!("the authorized terminal must split the ambiguous semicircle");
        };
        assert!(
            first
                .retained_facts
                .directed_sweep_angle
                .certified()
                .is_none()
        );
        assert!(first.retained_facts.sweep_kind.certified().is_none());
        assert_eq!(
            first
                .point_at_sweep_fraction(&half, &CurveContext::STRICT)
                .unwrap(),
            Classification::Uncertain(crate::UncertaintyReason::RealSign)
        );
    }
}
