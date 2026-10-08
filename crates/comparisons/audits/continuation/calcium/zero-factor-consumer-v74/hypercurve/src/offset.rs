//! Native line/arc parallel primitives and exact offset options.
//!
//! Topology-producing offset and stroke operations belong to [`crate::CurveRegion2`].

use hyperreal::{Real, RealSign};

use crate::classify::{is_zero, real_sign};
use crate::segment::{CircularArc2, LineSeg2, Segment2};
use crate::{Classification, CurveContext, CurveError, CurveResult, Point2, UncertaintyReason};

/// Corner construction for an exact signed region offset.
///
/// The style applies to outward corner joins. Inward raw joins are allowed to
/// cross and are removed by the authoritative region regularization pass.
#[derive(Clone, Debug, PartialEq)]
pub enum OffsetCornerStyle2 {
    /// Follow the radius circle centered at the authored source vertex.
    Round,
    /// Connect the two exact parallel endpoints by a line segment.
    Bevel,
    /// Meet exact tangent supports when the dimensionless miter ratio does not
    /// exceed `limit`; otherwise fall back deterministically to [`Self::Bevel`].
    Miter { limit: Real },
}

pub(crate) fn validate_offset_corner_style(
    style: &OffsetCornerStyle2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let OffsetCornerStyle2::Miter { limit } = style else {
        return Ok(Classification::Decided(()));
    };
    match real_sign(limit, policy) {
        Some(RealSign::Positive | RealSign::Zero) => Ok(Classification::Decided(())),
        Some(RealSign::Negative) => Err(CurveError::InvalidOffsetOptions),
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

/// Endpoint cap style for [`crate::CurveRegion2::stroke_path`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OffsetCap {
    /// Connect left and right traces with circular arcs centered on endpoints.
    Round,
    /// Connect left and right traces directly at each endpoint.
    Butt,
    /// Extend each trace by one half-width along endpoint tangents before
    /// adding straight endpoint connectors.
    Square,
}

impl LineSeg2 {
    /// Returns the constant-distance segment on this segment's left side.
    ///
    /// The offset direction is the normalized left normal `(-dy, dx) / length`.
    /// This is the primitive line-profile case used by profile offset
    /// algorithms; higher-level curve-string offsetting must still add joins,
    /// trim self-intersections, and rebuild topology. See profile-offset construction, for the line/arc
    /// primitive plus trim-and-join framing used by many CAD offset pipelines.
    pub fn offset_left(&self, distance: Real) -> CurveResult<Self> {
        let (dx, dy) = self.delta();
        let (unit_x, unit_y) = unit_direction_for_delta(&dx, &dy)?;
        let normal_x = -unit_y;
        let normal_y = unit_x;
        let offset_x = &normal_x * &distance;
        let offset_y = &normal_y * &distance;

        self.offset_between(
            self.start().translated(offset_x.clone(), offset_y.clone()),
            self.end().translated(offset_x, offset_y),
            distance,
        )
    }
}

impl CircularArc2 {
    pub(crate) fn left_offset_radius_scale(&self, distance: &Real) -> CurveResult<Real> {
        let radius = self.radius_squared().sqrt()?;
        if self.is_clockwise() {
            (&radius + distance) / radius
        } else {
            (&radius - distance) / radius
        }
        .map_err(Into::into)
    }

    /// Returns the constant-distance arc on this arc's left side.
    ///
    /// Counter-clockwise arcs have their left normal on the circle interior, so
    /// a positive offset decreases radius. Clockwise arcs have their left normal
    /// on the exterior, so a positive offset increases radius. Radius collapse
    /// and radius sign reversal are returned as explicit uncertainty because
    /// the primitive arc no longer has a valid circular-arc image at that
    /// distance. This concentric-arc primitive is one step in a complete profile
    /// offset pipeline.
    pub fn offset_left(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let radius_scale = self.left_offset_radius_scale(&distance)?;

        match real_sign(&radius_scale, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }

        let offset = Self::try_from_center_with_bulge(
            scale_from_center(self.start(), self.center(), &radius_scale),
            scale_from_center(self.end(), self.center(), &radius_scale),
            self.center().clone(),
            self.is_clockwise(),
            self.bulge().cloned(),
        )?;
        Ok(Classification::Decided(offset))
    }
}

impl Segment2 {
    /// Returns this segment's left-side primitive offset.
    ///
    /// Lines always produce a translated line. Arcs produce a concentric arc
    /// when the requested distance leaves a positive radius; radius collapse or
    /// reversal is reported as uncertainty instead of fabricating degenerate
    /// topology.
    pub fn offset_left(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match self {
            Self::Line(line) => line
                .offset_left(distance)
                .map(Self::Line)
                .map(Classification::Decided),
            Self::Arc(arc) => arc
                .offset_left(distance, policy)
                .map(|arc| arc.map(Segment2::Arc)),
        }
    }
}

pub(crate) fn scale_from_center(point: &Point2, center: &Point2, scale: &Real) -> Point2 {
    let radius = point.delta_from(center);
    Point2::new(
        center.x() + (&radius.0 * scale),
        center.y() + (&radius.1 * scale),
    )
}

fn unit_direction_for_delta(dx: &Real, dy: &Real) -> CurveResult<(Real, Real)> {
    let policy = CurveContext::STRICT;
    let dx_sign = real_sign(dx, &policy);
    let dy_sign = real_sign(dy, &policy);
    if is_zero(&(dx * dx - dy * dy), &policy) == Some(true)
        && matches!(dx_sign, Some(RealSign::Positive | RealSign::Negative))
        && matches!(dy_sign, Some(RealSign::Positive | RealSign::Negative))
    {
        let diagonal = (Real::from(2_i8).sqrt()? / Real::from(2_i8))?;
        let signed = |sign| match sign {
            Some(RealSign::Negative) => -diagonal.clone(),
            Some(RealSign::Positive) => diagonal.clone(),
            _ => unreachable!("nonzero diagonal component signs were certified"),
        };
        return Ok((signed(dx_sign), signed(dy_sign)));
    }
    let length = Real::dot2_refs([dx, dy], [dx, dy]).sqrt()?;
    Ok(((dx / &length)?, (dy / &length)?))
}

pub(crate) fn line_support_intersection(
    previous: &LineSeg2,
    next: &LineSeg2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Point2>>> {
    let (rx, ry) = previous.delta();
    let (sx, sy) = next.delta();
    let denominator = cross(&rx, &ry, &sx, &sy);

    match real_sign(&denominator, policy) {
        Some(RealSign::Zero) => Ok(Classification::Decided(None)),
        Some(RealSign::Positive | RealSign::Negative) => {
            let qmp = next.start().delta_from(previous.start());
            let numerator = cross(&qmp.0, &qmp.1, &sx, &sy);
            let t = numerator * denominator.inverse_ref_assuming_nonzero()?;
            Ok(Classification::Decided(Some(previous.point_at(t))))
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn cross(ax: &Real, ay: &Real, bx: &Real, by: &Real) -> Real {
    (ax * by) - (ay * bx)
}
