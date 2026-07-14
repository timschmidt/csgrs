//! Line and circular-arc segment primitives.

use hyperreal::{Real, RealSign, ZeroKnowledge as ZeroStatus};
use std::{
    cell::{OnceCell, RefCell},
    rc::Rc,
};

use crate::classify::{
    LineSide, classify_oriented_line, compare_reals, in_closed_unit_interval, is_zero,
};
use crate::{Classification, CurveError, CurvePolicy, CurveResult, ParamRange, Point2};
use std::cmp::Ordering;

/// A finite line segment.
#[derive(Clone, Debug)]
pub struct LineSeg2 {
    start: Point2,
    end: Point2,
    support: Option<Rc<LineSupport2>>,
    support_range: Option<ParamRange>,
    offset_provenance: Option<Rc<LineOffsetProvenance2>>,
}

impl PartialEq for LineSeg2 {
    fn eq(&self, other: &Self) -> bool {
        self.start == other.start && self.end == other.end && self.support == other.support
    }
}

#[derive(Debug, PartialEq)]
struct LineSupport2 {
    start: Point2,
    end: Point2,
}

#[derive(Debug, PartialEq)]
struct LineOffsetProvenance2 {
    source: Rc<LineSupport2>,
    left_distance: Real,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum RetainedLineRelation2 {
    Coincident,
    ParallelDistinct,
    Uncertain,
}

impl LineSeg2 {
    /// Constructs a line segment and rejects equal endpoints when provable.
    pub fn try_new(start: Point2, end: Point2) -> CurveResult<Self> {
        if start == end || start.distance_squared(&end).zero_status() == ZeroStatus::Zero {
            return Err(CurveError::ZeroLengthLine);
        }
        Ok(Self {
            start,
            end,
            support: None,
            support_range: None,
            offset_provenance: None,
        })
    }

    /// Constructs a line segment without validating endpoint distinctness.
    pub fn new_unchecked(start: Point2, end: Point2) -> Self {
        Self {
            start,
            end,
            support: None,
            support_range: None,
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
        self.support.as_ref().map_or_else(
            || self.delta(),
            |support| support.end.delta_from(&support.start),
        )
    }

    pub(crate) const fn has_retained_support(&self) -> bool {
        self.support.is_some()
    }

    pub(crate) fn support_start(&self) -> &Point2 {
        self.support
            .as_ref()
            .map_or(&self.start, |support| &support.start)
    }

    pub(crate) fn fragment_between(&self, start: Point2, end: Point2) -> CurveResult<Self> {
        if start == end || start.distance_squared(&end).zero_status() == ZeroStatus::Zero {
            return Err(CurveError::ZeroLengthLine);
        }
        let support = self.support.clone().or_else(|| {
            Some(Rc::new(LineSupport2 {
                start: self.start.clone(),
                end: self.end.clone(),
            }))
        });
        Ok(Self {
            start,
            end,
            support,
            support_range: None,
            offset_provenance: self.offset_provenance.clone(),
        })
    }

    pub(crate) fn fragment_between_with_source_range(
        &self,
        start: Point2,
        end: Point2,
        source_range: ParamRange,
    ) -> CurveResult<Self> {
        if start == end || start.distance_squared(&end).zero_status() == ZeroStatus::Zero {
            return Err(CurveError::ZeroLengthLine);
        }
        let support = self.support.clone().or_else(|| {
            Some(Rc::new(LineSupport2 {
                start: self.start.clone(),
                end: self.end.clone(),
            }))
        });
        let support_range = self
            .support_range
            .as_ref()
            .map_or(source_range.clone(), |parent| {
                let width = parent.end() - parent.start();
                ParamRange::new(
                    parent.start() + &width * source_range.start(),
                    parent.start() + width * source_range.end(),
                )
            });
        Ok(Self {
            start,
            end,
            support,
            support_range: Some(support_range),
            offset_provenance: self.offset_provenance.clone(),
        })
    }

    pub(crate) fn retained_support_ranges_decided_disjoint(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Option<bool> {
        let first_support = self.support.as_ref()?;
        let second_support = other.support.as_ref()?;
        if !Rc::ptr_eq(first_support, second_support) {
            return None;
        }
        let first = self.support_range.as_ref()?;
        let second = other.support_range.as_ref()?;
        let (first_start, first_end) = ordered_range(first, policy)?;
        let (second_start, second_end) = ordered_range(second, policy)?;
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
                source: self.support.clone().unwrap_or_else(|| {
                    Rc::new(LineSupport2 {
                        start: self.start.clone(),
                        end: self.end.clone(),
                    })
                }),
                left_distance: distance,
            },
            Some(provenance) => LineOffsetProvenance2 {
                source: provenance.source.clone(),
                left_distance: &provenance.left_distance + &distance,
            },
        };
        offset.offset_provenance = Some(Rc::new(provenance));
        Ok(offset)
    }

    pub(crate) fn retained_offset_relation(
        &self,
        other: &Self,
        policy: &CurvePolicy,
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
        if start == end || start.distance_squared(&end).zero_status() == ZeroStatus::Zero {
            return Err(CurveError::ZeroLengthLine);
        }
        let support = self.support.as_ref().map(|support| {
            Rc::new(LineSupport2 {
                start: map(&support.start),
                end: map(&support.end),
            })
        });
        Ok(Self {
            start,
            end,
            support,
            support_range: self.support_range.clone(),
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
        self.start.lerp(&self.end, t)
    }

    /// Returns this segment with traversal direction reversed.
    pub fn reversed(&self) -> Self {
        let offset_provenance = self.offset_provenance.as_ref().map(|provenance| {
            Rc::new(LineOffsetProvenance2 {
                source: Rc::new(LineSupport2 {
                    start: provenance.source.end.clone(),
                    end: provenance.source.start.clone(),
                }),
                left_distance: -provenance.left_distance.clone(),
            })
        });
        Self {
            start: self.end.clone(),
            end: self.start.clone(),
            support: self.support.clone(),
            support_range: self
                .support_range
                .as_ref()
                .map(|range| ParamRange::new(range.end().clone(), range.start().clone())),
            offset_provenance,
        }
    }

    /// Classifies a point relative to this oriented line segment's supporting line.
    pub fn classify_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<LineSide> {
        let support_start = self.support_start();
        let support_end = self
            .support
            .as_ref()
            .map_or(&self.end, |support| &support.end);
        classify_oriented_line(support_start, support_end, point, policy)
    }

    /// Prepares this line segment for repeated supporting-line classifications.
    ///
    /// The prepared view caches segment facts and predicate endpoint
    /// conversion, but it delegates every sidedness branch to the same exact
    /// predicate policy as [`LineSeg2::classify_point`].
    pub fn prepare_topology_queries(&self) -> crate::PreparedLineSeg2<'_> {
        crate::PreparedLineSeg2::from_line_segment(self)
    }

    /// Classifies whether a point lies on this finite line segment.
    pub fn contains_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        match self.classify_point(point, policy) {
            Classification::Decided(LineSide::On) => {}
            Classification::Decided(_) => return Classification::Decided(false),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
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
        crate::facts::line_segment_facts(self)
    }
}

/// A finite circular arc segment.
#[derive(Clone, Debug)]
pub struct CircularArc2 {
    start: Point2,
    end: Point2,
    center: Point2,
    radius_squared: Real,
    endpoints_on_stored_circle: bool,
    clockwise: bool,
    bulge: Option<Real>,
    pub(crate) retained_facts: Rc<CircularArcRetainedFacts2>,
}

#[derive(Debug, Default)]
pub(crate) struct CircularArcRetainedFacts2 {
    pub(crate) sweep_kind: OnceCell<crate::ExactCurveResult<crate::arc_bezier::ArcSweepKind>>,
    pub(crate) bezier_decomposition:
        OnceCell<crate::ExactCurveResult<crate::CircularArcBezierDecomposition2>>,
    representative_point: OnceCell<CurveResult<Classification<Point2>>>,
    directed_sweep_angle: OnceCell<CurveResult<Classification<Real>>>,
    parameter_lineage: OnceCell<Box<CircularArcParameterLineage2>>,
    parameter_witnesses: OnceCell<Box<RefCell<Vec<CircularArcParameterWitness2>>>>,
    fragments: OnceCell<Box<RefCell<Vec<CircularArcFragmentWitness2>>>>,
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
        self.start == other.start
            && self.end == other.end
            && self.center == other.center
            && self.radius_squared == other.radius_squared
            && self.clockwise == other.clockwise
            && self.bulge == other.bulge
    }
}

impl CircularArc2 {
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

        Ok(Self {
            start,
            end,
            center,
            radius_squared: start_radius_squared,
            endpoints_on_stored_circle,
            clockwise,
            bulge: None,
            retained_facts: Rc::new(CircularArcRetainedFacts2::default()),
        })
    }

    pub(crate) fn new_unchecked_with_radius(
        start: Point2,
        end: Point2,
        center: Point2,
        radius_squared: Real,
        clockwise: bool,
        bulge: Option<Real>,
    ) -> Self {
        Self {
            start,
            end,
            center,
            radius_squared,
            endpoints_on_stored_circle: false,
            clockwise,
            bulge,
            retained_facts: Rc::new(CircularArcRetainedFacts2 {
                sweep_kind: OnceCell::new(),
                bezier_decomposition: OnceCell::new(),
                representative_point: OnceCell::new(),
                directed_sweep_angle: OnceCell::new(),
                parameter_lineage: OnceCell::new(),
                parameter_witnesses: OnceCell::new(),
                fragments: OnceCell::new(),
            }),
        }
    }

    pub(crate) fn new_with_certified_radius(
        start: Point2,
        end: Point2,
        center: Point2,
        radius_squared: Real,
        clockwise: bool,
        bulge: Option<Real>,
    ) -> Self {
        Self {
            start,
            end,
            center,
            radius_squared,
            endpoints_on_stored_circle: true,
            clockwise,
            bulge,
            retained_facts: Rc::new(CircularArcRetainedFacts2 {
                sweep_kind: OnceCell::new(),
                bezier_decomposition: OnceCell::new(),
                representative_point: OnceCell::new(),
                directed_sweep_angle: OnceCell::new(),
                parameter_lineage: OnceCell::new(),
                parameter_witnesses: OnceCell::new(),
                fragments: OnceCell::new(),
            }),
        }
    }

    pub(crate) fn try_from_center_with_bulge(
        start: Point2,
        end: Point2,
        center: Point2,
        clockwise: bool,
        bulge: Option<Real>,
    ) -> CurveResult<Self> {
        let mut arc = Self::try_from_center(start, end, center, clockwise)?;
        arc.bulge = bulge;
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
        arc.bulge = Some(bulge);
        Ok(arc)
    }

    /// Returns the arc start point.
    pub const fn start(&self) -> &Point2 {
        &self.start
    }

    /// Returns the arc end point.
    pub const fn end(&self) -> &Point2 {
        &self.end
    }

    /// Returns the arc center.
    pub const fn center(&self) -> &Point2 {
        &self.center
    }

    /// Returns the squared radius.
    pub fn radius_squared(&self) -> Real {
        self.radius_squared.clone()
    }

    /// Returns the stored squared radius by reference.
    pub const fn radius_squared_ref(&self) -> &Real {
        &self.radius_squared
    }

    pub(crate) const fn endpoints_on_stored_circle_are_certified(&self) -> bool {
        self.endpoints_on_stored_circle
    }

    /// Returns whether this arc travels clockwise from start to end.
    pub const fn is_clockwise(&self) -> bool {
        self.clockwise
    }

    /// Returns the source bulge when this arc was constructed from one.
    pub const fn bulge(&self) -> Option<&Real> {
        self.bulge.as_ref()
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
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        if point_matches_arc_endpoint(self, point, policy) == Some(true) {
            return Classification::Decided(true);
        }

        let sweep_kind = match crate::arc_bezier::classify_sweep(self, None) {
            Ok(kind) => kind,
            Err(crate::ExactCurveError::Blocked(blocker)) => {
                return Classification::Uncertain(blocker.reason());
            }
            Err(crate::ExactCurveError::Invalid { .. }) => {
                return Classification::Uncertain(crate::UncertaintyReason::Predicate);
            }
        };
        if sweep_kind == crate::arc_bezier::ArcSweepKind::FullCircle {
            return Classification::Decided(true);
        }

        let start_side = classify_oriented_line(&self.center, &self.start, point, policy);
        let end_side = classify_oriented_line(&self.center, &self.end, point, policy);
        let (Classification::Decided(start_side), Classification::Decided(end_side)) =
            (start_side, end_side)
        else {
            return Classification::Uncertain(crate::UncertaintyReason::Predicate);
        };

        self.contains_classified_sweep_sides(start_side, end_side, sweep_kind)
    }

    pub(crate) fn contains_classified_sweep_sides(
        &self,
        start_side: LineSide,
        end_side: LineSide,
        sweep_kind: crate::arc_bezier::ArcSweepKind,
    ) -> Classification<bool> {
        let start_contains = if self.clockwise {
            matches!(start_side, LineSide::Right | LineSide::On)
        } else {
            matches!(start_side, LineSide::Left | LineSide::On)
        };
        let end_contains = if self.clockwise {
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
    pub fn contains_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        if point_matches_arc_endpoint(self, point, policy) == Some(true) {
            return Classification::Decided(true);
        }
        let radius_delta = point.distance_squared(&self.center) - self.radius_squared();
        match is_zero(&radius_delta, policy) {
            Some(false) => Classification::Decided(false),
            Some(true) => self.contains_sweep_point(point, policy),
            None => Classification::Uncertain(crate::UncertaintyReason::RealSign),
        }
    }

    /// Prepares this arc for repeated sweep and point-on-arc classifications.
    ///
    /// The prepared view caches the two radial supporting-line predicates used
    /// by [`CircularArc2::contains_sweep_point`] and delegates radius equality
    /// checks to the same exact scalar policy as [`CircularArc2::contains_point`].
    pub fn prepare_topology_queries(&self) -> crate::PreparedCircularArc2<'_> {
        crate::PreparedCircularArc2::from_circular_arc(self)
    }

    /// Returns conservative structural facts for this arc.
    ///
    /// These facts can schedule future circle/arc exact kernels while leaving
    /// topological decisions to certified predicates and exact sign queries.
    pub fn structural_facts(&self) -> crate::CircularArc2Facts {
        crate::facts::circular_arc_facts(self)
    }

    /// Returns a point in the interior of this arc's supported sweep.
    ///
    /// The point is the exact midpoint of the requested minor, semicircular,
    /// major, or full-circle traversal. Arc fragments retain their source
    /// angular parameterization, so repeated and nested trims evaluate this
    /// point without rebuilding nested trigonometric rotations.
    pub fn representative_point(
        &self,
        _policy: &CurvePolicy,
    ) -> CurveResult<Classification<Point2>> {
        match self.retained_representative_point() {
            Ok(Classification::Decided(point)) => Ok(Classification::Decided(point.clone())),
            Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(*reason)),
            Err(error) => Err(error.clone()),
        }
    }

    pub(crate) fn retained_representative_point(&self) -> &CurveResult<Classification<Point2>> {
        self.retained_facts
            .representative_point
            .get_or_init(|| self.compute_representative_point())
    }

    fn compute_representative_point(&self) -> CurveResult<Classification<Point2>> {
        let half = (Real::one() / Real::from(2_i8))?;
        if self.retained_facts.parameter_lineage.get().is_some() {
            return self.point_at_sweep_fraction(&half, &CurvePolicy::certified());
        }
        match self
            .rational_bezier_decomposition()
            .and_then(|decomposition| decomposition.point_at(&half))
        {
            Ok(point) => Ok(Classification::Decided(point)),
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
        policy: &CurvePolicy,
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
        policy: &CurvePolicy,
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
            Some(Ordering::Equal) => return Ok(Classification::Decided(self.start.clone())),
            Some(_) => {}
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
        match compare_reals(fraction, &Real::one(), policy) {
            Some(Ordering::Equal) => return Ok(Classification::Decided(self.end.clone())),
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

        let sweep_angle = match self.retained_directed_sweep_angle() {
            Ok(Classification::Decided(angle)) => angle,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(*reason));
            }
            Err(error) => return Err(error.clone()),
        };
        let (radial, traversal_angle) =
            if let Some(lineage) = self.retained_facts.parameter_lineage.get() {
                let root_width = lineage.root_range.end() - lineage.root_range.start();
                let root_fraction = lineage.root_range.start() + &(root_width * fraction);
                (
                    lineage.root_start.delta_from(&self.center),
                    &lineage.root_sweep_angle * root_fraction,
                )
            } else {
                (self.start.delta_from(&self.center), sweep_angle * fraction)
            };
        let signed_angle = if self.clockwise {
            -traversal_angle
        } else {
            traversal_angle
        };
        let cosine = signed_angle.clone().cos();
        let sine = signed_angle.sin();
        Ok(Classification::Decided(Point2::new(
            self.center.x() + (&radial.0 * &cosine) - (&radial.1 * &sine),
            self.center.y() + (&radial.0 * sine) + (&radial.1 * cosine),
        )))
    }

    fn retained_directed_sweep_angle(&self) -> &CurveResult<Classification<Real>> {
        self.retained_facts
            .directed_sweep_angle
            .get_or_init(|| self.compute_directed_sweep_angle())
    }

    fn retained_parameter_witness(&self, parameter: &Real) -> Option<Point2> {
        let witnesses = self.retained_facts.parameter_witnesses.get()?;
        witnesses
            .borrow()
            .iter()
            .find(|witness| witness.parameter == *parameter)
            .map(|witness| witness.point.clone())
    }

    fn retain_parameter_witness(&self, parameter: &Real, point: &Point2) {
        let witnesses = self
            .retained_facts
            .parameter_witnesses
            .get_or_init(|| Box::new(RefCell::new(Vec::new())));
        let mut witnesses = witnesses.borrow_mut();
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

    fn compute_directed_sweep_angle(&self) -> CurveResult<Classification<Real>> {
        let sweep_kind = match crate::arc_bezier::classify_sweep(self, None) {
            Ok(kind) => kind,
            Err(crate::ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            Err(crate::ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
        };
        if sweep_kind == crate::arc_bezier::ArcSweepKind::FullCircle {
            return Ok(Classification::Decided(Real::tau()));
        }
        directed_radial_angle(self, self.end(), &CurvePolicy::certified())
    }

    pub(crate) fn fragment_between_sweep_range(
        &self,
        start: Point2,
        end: Point2,
        source_range: &ParamRange,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if let Some(fragment) = self.retained_fragment(source_range, &start, &end) {
            return Ok(Classification::Decided(fragment));
        }
        let (root_start, root_sweep_angle, parent_root_range) =
            if let Some(lineage) = self.retained_facts.parameter_lineage.get() {
                (
                    lineage.root_start.clone(),
                    lineage.root_sweep_angle.clone(),
                    lineage.root_range.clone(),
                )
            } else {
                let source_sweep = match self.retained_directed_sweep_angle() {
                    Ok(Classification::Decided(angle)) => angle.clone(),
                    Ok(Classification::Uncertain(reason)) => {
                        return Ok(Classification::Uncertain(*reason));
                    }
                    Err(error) => return Err(error.clone()),
                };
                (
                    self.start.clone(),
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
            self.center.clone(),
            self.radius_squared(),
            self.clockwise,
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
        let _ = fragment
            .retained_facts
            .directed_sweep_angle
            .set(Ok(Classification::Decided(fragment_sweep.clone())));
        if let Some(kind) = sweep_kind_from_directed_angle(&fragment_sweep, policy) {
            let _ = fragment.retained_facts.sweep_kind.set(Ok(kind));
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
            .borrow()
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
            .get_or_init(|| Box::new(RefCell::new(Vec::new())));
        let mut fragments = fragments.borrow_mut();
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
        policy: &CurvePolicy,
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
        let total_angle = match self.retained_directed_sweep_angle() {
            Ok(Classification::Decided(angle)) => angle.clone(),
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(*reason));
            }
            Err(error) => return Err(error.clone()),
        };
        let parameter = (point_angle / total_angle).map_err(CurveError::from)?;
        self.retain_parameter_witness(&parameter, point);
        Ok(Classification::Decided(parameter))
    }

    /// Returns this arc with traversal direction reversed.
    pub fn reversed(&self) -> Self {
        Self {
            start: self.end.clone(),
            end: self.start.clone(),
            center: self.center.clone(),
            radius_squared: self.radius_squared.clone(),
            endpoints_on_stored_circle: self.endpoints_on_stored_circle,
            clockwise: !self.clockwise,
            bulge: self.bulge.as_ref().map(|bulge| -bulge.clone()),
            retained_facts: Rc::new(CircularArcRetainedFacts2::default()),
        }
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
    pub const fn start(&self) -> &Point2 {
        match self {
            Self::Line(line) => line.start(),
            Self::Arc(arc) => arc.start(),
        }
    }

    /// Returns the segment end point.
    pub const fn end(&self) -> &Point2 {
        match self {
            Self::Line(line) => line.end(),
            Self::Arc(arc) => arc.end(),
        }
    }

    /// Classifies whether a point lies on this finite segment.
    pub fn contains_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
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
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Point2>> {
        let half = (Real::one() / Real::from(2_i8))?;
        self.point_at(&half, policy)
    }

    /// Evaluates this segment at a normalized traversal parameter in `[0, 1]`.
    pub fn point_at(
        &self,
        parameter: &Real,
        policy: &CurvePolicy,
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
            Self::Arc(arc) => match arc
                .rational_bezier_decomposition()
                .and_then(|decomposition| decomposition.point_at(parameter))
            {
                Ok(point) => Ok(Classification::Decided(point)),
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
}

#[allow(clippy::large_enum_variant)]
enum ParameterOnLine {
    Decided(Real),
    Uncertain(crate::UncertaintyReason),
}

fn parameter_on_line(line: &LineSeg2, point: &Point2, policy: &CurvePolicy) -> ParameterOnLine {
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

    // Bulge sign chooses the arc sweep orientation, so it is a topology
    // decision rather than an IO/display choice. Use bounded exact-real
    // refinement here instead of a primitive-float fallback, matching the exactness model's
    // requirement that combinatorial decisions be separated from approximate
    // views.
    match bulge.refine_sign_until(-4096) {
        Some(RealSign::Negative) => Ok(true),
        Some(RealSign::Positive) => Ok(false),
        Some(RealSign::Zero) => Err(CurveError::AmbiguousBulge),
        None => Err(CurveError::AmbiguousBulge),
    }
}

fn point_matches_arc_endpoint(
    arc: &CircularArc2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<bool> {
    let start_distance = point.distance_squared(&arc.start);
    if crate::classify::is_zero(&start_distance, policy)? {
        return Some(true);
    }
    let end_distance = point.distance_squared(&arc.end);
    crate::classify::is_zero(&end_distance, policy)
}

fn ordered_range<'a>(range: &'a ParamRange, policy: &CurvePolicy) -> Option<(&'a Real, &'a Real)> {
    match compare_reals(range.start(), range.end(), policy)? {
        Ordering::Less | Ordering::Equal => Some((range.start(), range.end())),
        Ordering::Greater => Some((range.end(), range.start())),
    }
}

fn points_equal(left: &Point2, right: &Point2, policy: &CurvePolicy) -> Option<bool> {
    if left == right {
        return Some(true);
    }
    crate::classify::is_zero(&left.distance_squared(right), policy)
}

fn directed_radial_angle(
    arc: &CircularArc2,
    point: &Point2,
    policy: &CurvePolicy,
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
    let angle = match cross_sign {
        RealSign::Positive => directed_cross.atan2(dot),
        RealSign::Negative => directed_cross.atan2(dot) + Real::tau(),
        RealSign::Zero => match crate::classify::real_sign(&dot, policy) {
            Some(RealSign::Positive) => Real::zero(),
            Some(RealSign::Negative) => Real::pi(),
            Some(RealSign::Zero) => {
                return Err(CurveError::InvalidCurveParameter);
            }
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::RealSign,
                ));
            }
        },
    };
    Ok(Classification::Decided(angle))
}

fn sweep_kind_from_directed_angle(
    angle: &Real,
    policy: &CurvePolicy,
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
