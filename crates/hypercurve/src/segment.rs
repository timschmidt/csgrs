//! Line and circular-arc segment primitives.

use hyperreal::{Real, RealSign, ZeroKnowledge as ZeroStatus};
use std::rc::Rc;

use crate::classify::{LineSide, classify_oriented_line, in_closed_unit_interval, is_zero};
use crate::{Classification, CurveError, CurvePolicy, CurveResult, Point2};

/// A finite line segment.
#[derive(Clone, Debug, PartialEq)]
pub struct LineSeg2 {
    start: Point2,
    end: Point2,
    support: Option<Rc<LineSupport2>>,
}

#[derive(Debug, PartialEq)]
struct LineSupport2 {
    start: Point2,
    end: Point2,
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
        })
    }

    /// Constructs a line segment without validating endpoint distinctness.
    pub fn new_unchecked(start: Point2, end: Point2) -> Self {
        Self {
            start,
            end,
            support: None,
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
        })
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
        Self {
            start: self.end.clone(),
            end: self.start.clone(),
            support: self.support.clone(),
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
#[derive(Clone, Debug, PartialEq)]
pub struct CircularArc2 {
    start: Point2,
    end: Point2,
    center: Point2,
    radius_squared: Real,
    clockwise: bool,
    bulge: Option<Real>,
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
        if mismatch.zero_status() == ZeroStatus::NonZero {
            return Err(CurveError::RadiusMismatch);
        }

        Ok(Self {
            start,
            end,
            center,
            radius_squared: start_radius_squared,
            clockwise,
            bulge: None,
        })
    }

    pub(crate) const fn new_unchecked_with_radius(
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
            clockwise,
            bulge,
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
    /// This assumes the current MVP arc model: a circular arc is the minor or
    /// semicircular sweep implied by endpoints plus orientation. The point does
    /// not have to be on the circle; callers that need point-on-arc semantics
    /// should also compare squared distance to [`CircularArc2::radius_squared`].
    /// The half-plane tests are the finite-arc containment counterpart to the
    /// circle and arc primitive tests catalogued by Schneider and Eberly,
    /// *Geometric Tools for Computer Graphics* (Morgan Kaufmann, 2002).
    pub fn contains_sweep_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        if point_matches_arc_endpoint(self, point, policy) == Some(true) {
            return Classification::Decided(true);
        }

        let start_side = classify_oriented_line(&self.center, &self.start, point, policy);
        let end_side = classify_oriented_line(&self.center, &self.end, point, policy);
        let (Classification::Decided(start_side), Classification::Decided(end_side)) =
            (start_side, end_side)
        else {
            return Classification::Uncertain(crate::UncertaintyReason::Predicate);
        };

        let contains = if self.clockwise {
            matches!(start_side, LineSide::Right | LineSide::On)
                && matches!(end_side, LineSide::Left | LineSide::On)
        } else {
            matches!(start_side, LineSide::Left | LineSide::On)
                && matches!(end_side, LineSide::Right | LineSide::On)
        };
        Classification::Decided(contains)
    }

    /// Classifies whether a point lies on this finite circular arc.
    pub fn contains_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
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
    /// The current arc model is intentionally restricted to minor and
    /// semicircular sweeps. For non-semicircles the midpoint is the normalized
    /// sum of the endpoint radius vectors. For semicircles the sum is zero, so
    /// the midpoint is the perpendicular radius selected by orientation.
    pub fn representative_point(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Point2>> {
        let start_radius = self.start.delta_from(&self.center);
        let end_radius = self.end.delta_from(&self.center);
        let sum_x = &start_radius.0 + &end_radius.0;
        let sum_y = &start_radius.1 + &end_radius.1;
        let sum_length_squared = (&sum_x * &sum_x) + (&sum_y * &sum_y);

        match is_zero(&sum_length_squared, policy) {
            Some(true) => {
                let (mid_x, mid_y) = if self.clockwise {
                    (start_radius.1, -start_radius.0)
                } else {
                    (-start_radius.1, start_radius.0)
                };
                Ok(Classification::Decided(Point2::new(
                    self.center.x() + mid_x,
                    self.center.y() + mid_y,
                )))
            }
            Some(false) => {
                let scale = (self.radius_squared() / &sum_length_squared)?.sqrt()?;
                Ok(Classification::Decided(Point2::new(
                    self.center.x() + (&sum_x * &scale),
                    self.center.y() + (&sum_y * &scale),
                )))
            }
            None => Ok(Classification::Uncertain(
                crate::UncertaintyReason::RealSign,
            )),
        }
    }

    /// Returns this arc with traversal direction reversed.
    pub fn reversed(&self) -> Self {
        Self {
            start: self.end.clone(),
            end: self.start.clone(),
            center: self.center.clone(),
            radius_squared: self.radius_squared.clone(),
            clockwise: !self.clockwise,
            bulge: self.bulge.as_ref().map(|bulge| -bulge.clone()),
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
        match self {
            Self::Line(line) => {
                let half = (Real::one() / Real::from(2_i8))?;
                Ok(Classification::Decided(line.point_at(half)))
            }
            Self::Arc(arc) => arc.representative_point(policy),
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
    // refinement here instead of a primitive-float fallback, matching Yap's
    // requirement that combinatorial decisions be separated from approximate
    // views. See Yap, "Towards Exact Geometric Computation," *Computational
    // Geometry* 7.1-2 (1997).
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
