//! Line and circular-arc segment primitives.

use hyperlattice::{Backend, DefaultBackend, Scalar, ScalarSign, ZeroStatus};

use crate::{CurveError, CurveResult, Point2};

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
