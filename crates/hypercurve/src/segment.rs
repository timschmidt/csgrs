//! Line and circular-arc segment primitives.

use hyperlattice::{Backend, DefaultBackend, Scalar, ScalarSign, ZeroStatus};

use crate::classify::{LineSide, classify_oriented_line, in_closed_unit_interval, is_zero};
use crate::{Classification, CurveError, CurvePolicy, CurveResult, Point2};

/// A finite line segment.
#[derive(Clone, Debug, PartialEq)]
pub struct LineSeg2<B: Backend = DefaultBackend> {
    start: Point2<B>,
    end: Point2<B>,
}

impl<B: Backend> LineSeg2<B> {
    /// Constructs a line segment and rejects equal endpoints when provable.
    pub fn try_new(start: Point2<B>, end: Point2<B>) -> CurveResult<Self> {
        if start.distance_squared(&end).zero_status() == ZeroStatus::Zero {
            return Err(CurveError::ZeroLengthLine);
        }
        Ok(Self { start, end })
    }

    /// Constructs a line segment without validating endpoint distinctness.
    pub const fn new_unchecked(start: Point2<B>, end: Point2<B>) -> Self {
        Self { start, end }
    }

    /// Returns the segment start point.
    pub const fn start(&self) -> &Point2<B> {
        &self.start
    }

    /// Returns the segment end point.
    pub const fn end(&self) -> &Point2<B> {
        &self.end
    }

    /// Returns `(end.x - start.x, end.y - start.y)`.
    pub fn delta(&self) -> (Scalar<B>, Scalar<B>) {
        self.end.delta_from(&self.start)
    }

    /// Returns squared segment length.
    pub fn length_squared(&self) -> Scalar<B> {
        self.start.distance_squared(&self.end)
    }

    /// Returns the point at affine parameter `t`, where `0` is start and `1` is end.
    pub fn point_at(&self, t: Scalar<B>) -> Point2<B> {
        self.start.lerp(&self.end, t)
    }

    /// Returns this segment with traversal direction reversed.
    pub fn reversed(&self) -> Self {
        Self {
            start: self.end.clone(),
            end: self.start.clone(),
        }
    }

    /// Classifies a point relative to this oriented line segment's supporting line.
    pub fn classify_point(
        &self,
        point: &Point2<B>,
        policy: &CurvePolicy,
    ) -> Classification<LineSide> {
        classify_oriented_line(&self.start, &self.end, point, policy)
    }

    /// Classifies whether a point lies on this finite line segment.
    pub fn contains_point(&self, point: &Point2<B>, policy: &CurvePolicy) -> Classification<bool> {
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
}

/// A finite circular arc segment.
#[derive(Clone, Debug, PartialEq)]
pub struct CircularArc2<B: Backend = DefaultBackend> {
    start: Point2<B>,
    end: Point2<B>,
    center: Point2<B>,
    radius_squared: Scalar<B>,
    clockwise: bool,
    bulge: Option<Scalar<B>>,
}

impl<B: Backend> CircularArc2<B> {
    /// Constructs a circular arc from endpoints, center, and orientation.
    pub fn try_from_center(
        start: Point2<B>,
        end: Point2<B>,
        center: Point2<B>,
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

    pub(crate) fn try_from_center_with_bulge(
        start: Point2<B>,
        end: Point2<B>,
        center: Point2<B>,
        clockwise: bool,
        bulge: Option<Scalar<B>>,
    ) -> CurveResult<Self> {
        let mut arc = Self::try_from_center(start, end, center, clockwise)?;
        arc.bulge = bulge;
        Ok(arc)
    }

    /// Constructs a circular arc from Cavalier/CAD bulge geometry.
    ///
    /// The formula keeps the center computation in rational operations:
    /// `center = midpoint + left_perp(chord) * ((1 - b^2) / (4b))`.
    pub fn from_bulge(start: Point2<B>, end: Point2<B>, bulge: Scalar<B>) -> CurveResult<Self> {
        if start.distance_squared(&end).zero_status() == ZeroStatus::Zero {
            return Err(CurveError::ZeroLengthLine);
        }

        let clockwise = clockwise_from_bulge(&bulge)?;
        let four_b = Scalar::<B>::from(4_i8) * &bulge;
        let b2 = &bulge * &bulge;
        let offset_factor = ((Scalar::<B>::one() - &b2) / four_b)?;
        let two = Scalar::<B>::from(2_i8);
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
    pub const fn start(&self) -> &Point2<B> {
        &self.start
    }

    /// Returns the arc end point.
    pub const fn end(&self) -> &Point2<B> {
        &self.end
    }

    /// Returns the arc center.
    pub const fn center(&self) -> &Point2<B> {
        &self.center
    }

    /// Returns the squared radius.
    pub fn radius_squared(&self) -> Scalar<B> {
        self.radius_squared.clone()
    }

    /// Returns whether this arc travels clockwise from start to end.
    pub const fn is_clockwise(&self) -> bool {
        self.clockwise
    }

    /// Returns the source bulge when this arc was constructed from one.
    pub const fn bulge(&self) -> Option<&Scalar<B>> {
        self.bulge.as_ref()
    }

    /// Classifies whether a point lies inside this arc's angular sweep.
    ///
    /// This assumes the current MVP arc model: a circular arc is the minor or
    /// semicircular sweep implied by endpoints plus orientation. The point does
    /// not have to be on the circle; callers that need point-on-arc semantics
    /// should also compare squared distance to [`CircularArc2::radius_squared`].
    pub fn contains_sweep_point(
        &self,
        point: &Point2<B>,
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
    pub fn contains_point(&self, point: &Point2<B>, policy: &CurvePolicy) -> Classification<bool> {
        let radius_delta = point.distance_squared(&self.center) - self.radius_squared();
        match is_zero(&radius_delta, policy) {
            Some(false) => Classification::Decided(false),
            Some(true) => self.contains_sweep_point(point, policy),
            None => Classification::Uncertain(crate::UncertaintyReason::ScalarSign),
        }
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
    ) -> CurveResult<Classification<Point2<B>>> {
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
                crate::UncertaintyReason::ScalarSign,
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
pub enum Segment2<B: Backend = DefaultBackend> {
    /// Straight line segment.
    Line(LineSeg2<B>),
    /// Circular arc segment.
    Arc(CircularArc2<B>),
}

impl<B: Backend> Segment2<B> {
    /// Constructs a native segment from a bulge value.
    ///
    /// Zero bulge maps to a line. Nonzero bulge maps to a circular arc.
    pub fn from_bulge(start: Point2<B>, end: Point2<B>, bulge: Scalar<B>) -> CurveResult<Self> {
        match bulge.zero_status() {
            ZeroStatus::Zero => LineSeg2::try_new(start, end).map(Self::Line),
            ZeroStatus::NonZero => CircularArc2::from_bulge(start, end, bulge).map(Self::Arc),
            ZeroStatus::Unknown => Err(CurveError::AmbiguousBulge),
        }
    }

    /// Constructs a segment from a Cavalier-compatible bulge.
    ///
    /// Cavalier's public semantics support single arc segments up to a half
    /// circle. Larger sweeps should be split before import.
    pub fn from_cavalier_bulge(
        start: Point2<B>,
        end: Point2<B>,
        bulge: Scalar<B>,
    ) -> CurveResult<Self> {
        reject_cavalier_unsupported_bulge(&bulge)?;
        Self::from_bulge(start, end, bulge)
    }

    /// Returns the segment start point.
    pub const fn start(&self) -> &Point2<B> {
        match self {
            Self::Line(line) => line.start(),
            Self::Arc(arc) => arc.start(),
        }
    }

    /// Returns the segment end point.
    pub const fn end(&self) -> &Point2<B> {
        match self {
            Self::Line(line) => line.end(),
            Self::Arc(arc) => arc.end(),
        }
    }

    /// Classifies whether a point lies on this finite segment.
    pub fn contains_point(&self, point: &Point2<B>, policy: &CurvePolicy) -> Classification<bool> {
        match self {
            Self::Line(line) => line.contains_point(point, policy),
            Self::Arc(arc) => arc.contains_point(point, policy),
        }
    }

    /// Returns a point in the interior of this segment.
    pub fn representative_point(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Point2<B>>> {
        match self {
            Self::Line(line) => {
                let half = (Scalar::<B>::one() / Scalar::<B>::from(2_i8))?;
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

enum ParameterOnLine<B: Backend> {
    Decided(Scalar<B>),
    Uncertain(crate::UncertaintyReason),
}

fn parameter_on_line<B: Backend>(
    line: &LineSeg2<B>,
    point: &Point2<B>,
    policy: &CurvePolicy,
) -> ParameterOnLine<B> {
    let (dx, dy) = line.delta();
    let delta = point.delta_from(line.start());

    match is_zero(&dx, policy) {
        Some(false) => match delta.0 / dx {
            Ok(t) => ParameterOnLine::Decided(t),
            Err(_) => ParameterOnLine::Uncertain(crate::UncertaintyReason::ScalarSign),
        },
        Some(true) => match delta.1 / dy {
            Ok(t) => ParameterOnLine::Decided(t),
            Err(_) => ParameterOnLine::Uncertain(crate::UncertaintyReason::ScalarSign),
        },
        None => match is_zero(&dy, policy) {
            Some(false) => match delta.1 / dy {
                Ok(t) => ParameterOnLine::Decided(t),
                Err(_) => ParameterOnLine::Uncertain(crate::UncertaintyReason::ScalarSign),
            },
            Some(true) => ParameterOnLine::Uncertain(crate::UncertaintyReason::ScalarSign),
            None => ParameterOnLine::Uncertain(crate::UncertaintyReason::ScalarSign),
        },
    }
}

fn clockwise_from_bulge<B: Backend>(bulge: &Scalar<B>) -> CurveResult<bool> {
    if let Some(sign) = bulge.structural_facts().sign {
        return match sign {
            ScalarSign::Negative => Ok(true),
            ScalarSign::Positive => Ok(false),
            ScalarSign::Zero => Err(CurveError::AmbiguousBulge),
        };
    }

    let approx = bulge.to_f64_approx().ok_or(CurveError::AmbiguousBulge)?;
    if approx < 0.0 {
        Ok(true)
    } else if approx > 0.0 {
        Ok(false)
    } else {
        Err(CurveError::AmbiguousBulge)
    }
}

fn reject_cavalier_unsupported_bulge<B: Backend>(bulge: &Scalar<B>) -> CurveResult<()> {
    if bulge.zero_status() == ZeroStatus::Zero {
        return Ok(());
    }

    let Some(approx) = bulge.to_f64_approx() else {
        return Ok(());
    };

    if approx.abs() > 1.0 {
        Err(CurveError::UnsupportedBulge)
    } else {
        Ok(())
    }
}

fn point_matches_arc_endpoint<B: Backend>(
    arc: &CircularArc2<B>,
    point: &Point2<B>,
    policy: &CurvePolicy,
) -> Option<bool> {
    let start_distance = point.distance_squared(&arc.start);
    if crate::classify::is_zero(&start_distance, policy)? {
        return Some(true);
    }
    let end_distance = point.distance_squared(&arc.end);
    crate::classify::is_zero(&end_distance, policy)
}
