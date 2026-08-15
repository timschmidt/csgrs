//! Native line/arc parallel primitives and private region-offset fast paths.
//!
//! Topology-producing offset and stroke operations belong to [`crate::CurveRegion2`].

use hyperreal::{Real, RealSign};
use std::cmp::Ordering;

use crate::classify::{compare_reals, is_zero, real_sign};
use crate::contour::{Contour2, FillRule};
use crate::curve_string::CurveString2;
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

impl CurveString2 {
    /// Returns a left offset of this open curve string with straight-line joins.
    ///
    /// This is a raw offset-construction layer, not a full offset engine. Each
    /// source segment is first replaced by its primitive parallel offset. Adjacent
    /// offset lines are mitered by intersecting their supporting lines; joins
    /// that cannot be mitered are connected by a circular arc centered at the
    /// original shared vertex. A complete profile offset pipeline still has to
    /// classify join style, trim self-intersections, and cap open endpoints.
    /// profile-offset construction,
    /// describe this staged primitive, join, and trim structure for
    /// two-dimensional profile offsets.
    pub fn offset_left_with_line_joins(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if is_zero(&distance, policy) == Some(true) {
            return Ok(Classification::Decided(self.clone()));
        }

        let offsets = match offset_segments_left(self.segments(), &distance, policy)? {
            Classification::Decided(offsets) => offsets,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let joined = match joined_offset_segments(
            self.segments(),
            &offsets,
            false,
            None,
            &distance,
            policy,
        )? {
            Classification::Decided(joined) => joined,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(checked_joined_curve_string(joined))
    }
}

impl Contour2 {
    pub(crate) fn offset_left_with_corner_style(
        &self,
        distance: Real,
        style: &OffsetCornerStyle2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if is_zero(&distance, policy) == Some(true) {
            return Ok(Classification::Decided(self.clone()));
        }
        let offsets = match offset_segments_left(self.segments(), &distance, policy)? {
            Classification::Decided(offsets) => offsets,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let joined = match joined_offset_segments(
            self.segments(),
            &offsets,
            true,
            Some(style),
            &distance,
            policy,
        )? {
            Classification::Decided(joined) => joined,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(checked_joined_contour(joined, self.fill_rule())
            .map(|offset| offset.retain_left_offset_from(self, distance, policy)))
    }

    /// Decides whether the requested authored joins are represented exactly by
    /// the line straight-skeleton wavefront.
    ///
    /// Convex contracting corners miter for every style, while a reflex corner
    /// may request an arc, bevel, or limited-miter fallback. Only connected and
    /// miter joins share the skeleton's moving-support boundary.
    pub(crate) fn offset_left_uses_line_wavefront_joins(
        &self,
        distance: Real,
        style: &OffsetCornerStyle2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if self
            .segments()
            .iter()
            .any(|segment| matches!(segment, Segment2::Arc(_)))
        {
            return Ok(Classification::Decided(false));
        }
        let offsets = match offset_segments_left(self.segments(), &distance, policy)? {
            Classification::Decided(offsets) => offsets,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        classified_offset_joins(
            self.segments(),
            &offsets,
            true,
            Some(style),
            &distance,
            policy,
        )
        .map(|classification| {
            classification.map(|joins| {
                joins
                    .iter()
                    .all(|join| matches!(join, OffsetJoin::Connected | OffsetJoin::Miter(_)))
            })
        })
    }
}

pub(crate) fn scale_from_center(point: &Point2, center: &Point2, scale: &Real) -> Point2 {
    let radius = point.delta_from(center);
    Point2::new(
        center.x() + (&radius.0 * scale),
        center.y() + (&radius.1 * scale),
    )
}

fn checked_joined_curve_string(segments: Vec<Segment2>) -> Classification<CurveString2> {
    CurveString2::try_new(segments)
        .map(Classification::Decided)
        .unwrap_or_else(classify_joined_topology_error)
}

fn checked_joined_contour(
    segments: Vec<Segment2>,
    fill_rule: FillRule,
) -> Classification<Contour2> {
    Contour2::try_new_with_fill_rule(segments, fill_rule)
        .map(Classification::Decided)
        .unwrap_or_else(classify_joined_topology_error)
}

fn classify_joined_topology_error<T>(error: CurveError) -> Classification<T> {
    match error {
        CurveError::DisconnectedCurveString => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        CurveError::AmbiguousCurveStringConnection => {
            Classification::Uncertain(UncertaintyReason::RealSign)
        }
        _ => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
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

fn offset_segments_left(
    segments: &[Segment2],
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<Segment2>>> {
    let mut offsets = Vec::with_capacity(segments.len());
    for segment in segments {
        match segment.offset_left(distance.clone(), policy)? {
            Classification::Decided(offset) => offsets.push(offset),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(offsets))
}

#[derive(Clone, Debug, PartialEq)]
enum OffsetJoin {
    Connected,
    Miter(Point2),
    Round { center: Point2 },
    Bevel,
}

fn joined_offset_segments(
    source: &[Segment2],
    offsets: &[Segment2],
    closed: bool,
    corner_style: Option<&OffsetCornerStyle2>,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<Segment2>>> {
    if offsets.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }
    if source.len() != offsets.len() {
        return Err(CurveError::Topology(
            "source and offset segment counts differ".into(),
        ));
    }

    let joins =
        match classified_offset_joins(source, offsets, closed, corner_style, distance, policy)? {
            Classification::Decided(joins) => joins,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

    let mut joined = Vec::with_capacity(offsets.len() + joins.len());
    for index in 0..offsets.len() {
        let start_override = start_miter_for_segment(index, offsets.len(), closed, &joins);
        let end_override = end_miter_for_segment(index, &joins);
        let adjusted = match adjust_offset_segment(
            &offsets[index],
            start_override.as_ref(),
            end_override.as_ref(),
        )? {
            Classification::Decided(segment) => segment,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let adjusted_end = adjusted.end().clone();
        joined.push(adjusted);

        let to = offsets[(index + 1) % offsets.len()].start().clone();
        match joins.get(index) {
            Some(OffsetJoin::Round { center }) => {
                match append_round_join_if_needed(&mut joined, &adjusted_end, &to, center, policy)?
                {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Some(OffsetJoin::Bevel) => {
                match append_bevel_join_if_needed(&mut joined, &adjusted_end, &to, policy)? {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Some(OffsetJoin::Connected | OffsetJoin::Miter(_)) | None => {}
        }
    }

    Ok(Classification::Decided(joined))
}

fn classified_offset_joins(
    source: &[Segment2],
    offsets: &[Segment2],
    closed: bool,
    corner_style: Option<&OffsetCornerStyle2>,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<OffsetJoin>>> {
    let join_count = if closed {
        offsets.len()
    } else {
        offsets.len().saturating_sub(1)
    };
    let mut joins = Vec::with_capacity(join_count);
    for index in 0..join_count {
        let next_index = (index + 1) % offsets.len();
        match classify_offset_join(
            &source[index],
            &source[next_index],
            &offsets[index],
            &offsets[next_index],
            corner_style,
            distance,
            policy,
        )? {
            Classification::Decided(join) => joins.push(join),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(joins))
}

fn classify_offset_join(
    source_previous: &Segment2,
    source_next: &Segment2,
    offset_previous: &Segment2,
    offset_next: &Segment2,
    corner_style: Option<&OffsetCornerStyle2>,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<OffsetJoin>> {
    match is_zero(
        &offset_previous.end().distance_squared(offset_next.start()),
        policy,
    ) {
        Some(true) => return Ok(Classification::Decided(OffsetJoin::Connected)),
        Some(false) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }

    let Some(style) = corner_style else {
        return classify_legacy_offset_join(
            source_previous,
            source_next,
            offset_previous,
            offset_next,
            policy,
        );
    };
    let inward = match offset_corner_is_inward(source_previous, source_next, distance, policy)? {
        Classification::Decided(inward) => inward,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if inward {
        return match (offset_previous, offset_next) {
            (Segment2::Line(previous), Segment2::Line(next)) => {
                match line_support_intersection(previous, next, policy)? {
                    Classification::Decided(Some(point)) => {
                        Ok(Classification::Decided(OffsetJoin::Miter(point)))
                    }
                    Classification::Decided(None) => Ok(Classification::Decided(OffsetJoin::Bevel)),
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                }
            }
            _ => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        };
    }

    match style {
        OffsetCornerStyle2::Round => Ok(Classification::Decided(round_join(
            source_previous,
            source_next,
        ))),
        OffsetCornerStyle2::Bevel => Ok(Classification::Decided(OffsetJoin::Bevel)),
        OffsetCornerStyle2::Miter { limit } => match (offset_previous, offset_next) {
            (Segment2::Line(previous), Segment2::Line(next)) => {
                match line_support_intersection(previous, next, policy)? {
                    Classification::Decided(Some(point)) => {
                        match miter_within_limit(
                            &point,
                            source_previous.end(),
                            distance,
                            limit,
                            policy,
                        ) {
                            Classification::Decided(true) => {
                                Ok(Classification::Decided(OffsetJoin::Miter(point)))
                            }
                            Classification::Decided(false) => {
                                Ok(Classification::Decided(OffsetJoin::Bevel))
                            }
                            Classification::Uncertain(reason) => {
                                Ok(Classification::Uncertain(reason))
                            }
                        }
                    }
                    Classification::Decided(None) => Ok(Classification::Decided(OffsetJoin::Bevel)),
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                }
            }
            _ => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        },
    }
}

fn classify_legacy_offset_join(
    source_previous: &Segment2,
    source_next: &Segment2,
    offset_previous: &Segment2,
    offset_next: &Segment2,
    policy: &CurveContext,
) -> CurveResult<Classification<OffsetJoin>> {
    match (offset_previous, offset_next) {
        (Segment2::Line(previous), Segment2::Line(next)) => {
            match line_support_intersection(previous, next, policy)? {
                Classification::Decided(Some(point)) => {
                    Ok(Classification::Decided(OffsetJoin::Miter(point)))
                }
                Classification::Decided(None) => Ok(Classification::Decided(round_join(
                    source_previous,
                    source_next,
                ))),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        }
        _ => Ok(Classification::Decided(round_join(
            source_previous,
            source_next,
        ))),
    }
}

fn offset_corner_is_inward(
    previous: &Segment2,
    next: &Segment2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let previous_tangent = segment_endpoint_tangent(previous, false);
    let next_tangent = segment_endpoint_tangent(next, true);
    let turn = cross(
        &previous_tangent.0,
        &previous_tangent.1,
        &next_tangent.0,
        &next_tangent.1,
    );
    Ok(match real_sign(&(turn * distance), policy) {
        Some(RealSign::Positive) => Classification::Decided(true),
        Some(RealSign::Negative | RealSign::Zero) => Classification::Decided(false),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    })
}

fn segment_endpoint_tangent(segment: &Segment2, start: bool) -> (Real, Real) {
    match segment {
        Segment2::Line(line) => line.delta(),
        Segment2::Arc(arc) => {
            let point = if start { arc.start() } else { arc.end() };
            let (rx, ry) = point.delta_from(arc.center());
            if arc.is_clockwise() {
                (ry, -rx)
            } else {
                (-ry, rx)
            }
        }
    }
}

fn miter_within_limit(
    miter: &Point2,
    source_vertex: &Point2,
    distance: &Real,
    limit: &Real,
    policy: &CurveContext,
) -> Classification<bool> {
    let miter_distance_squared = miter.distance_squared(source_vertex);
    let maximum_squared = distance * distance * limit * limit;
    match compare_reals(&miter_distance_squared, &maximum_squared, policy) {
        Some(Ordering::Less | Ordering::Equal) => Classification::Decided(true),
        Some(Ordering::Greater) => Classification::Decided(false),
        None => Classification::Uncertain(UncertaintyReason::Ordering),
    }
}

fn round_join(previous: &Segment2, next: &Segment2) -> OffsetJoin {
    let _ = next;
    OffsetJoin::Round {
        center: previous.end().clone(),
    }
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
            let t = (numerator / &denominator)?;
            Ok(Classification::Decided(Some(previous.point_at(t))))
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn start_miter_for_segment(
    index: usize,
    segment_count: usize,
    closed: bool,
    joins: &[OffsetJoin],
) -> Option<Point2> {
    if !closed && index == 0 {
        return None;
    }
    let join_index = if index == 0 {
        segment_count - 1
    } else {
        index - 1
    };
    match joins.get(join_index) {
        Some(OffsetJoin::Miter(point)) => Some(point.clone()),
        _ => None,
    }
}

fn end_miter_for_segment(index: usize, joins: &[OffsetJoin]) -> Option<Point2> {
    match joins.get(index) {
        Some(OffsetJoin::Miter(point)) => Some(point.clone()),
        _ => None,
    }
}

fn adjust_offset_segment(
    segment: &Segment2,
    start_override: Option<&Point2>,
    end_override: Option<&Point2>,
) -> CurveResult<Classification<Segment2>> {
    match segment {
        Segment2::Line(line) => {
            let start = start_override
                .cloned()
                .unwrap_or_else(|| line.start().clone());
            let end = end_override.cloned().unwrap_or_else(|| line.end().clone());
            match line.fragment_between(start, end) {
                Ok(line) => Ok(Classification::Decided(Segment2::Line(line))),
                Err(CurveError::ZeroLengthLine) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                }
                Err(error) => Err(error),
            }
        }
        Segment2::Arc(_) if start_override.is_some() || end_override.is_some() => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Segment2::Arc(arc) => Ok(Classification::Decided(Segment2::Arc(arc.clone()))),
    }
}

fn append_round_join_if_needed(
    joined: &mut Vec<Segment2>,
    from: &Point2,
    to: &Point2,
    center: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let distance = from.distance_squared(to);
    match is_zero(&distance, policy) {
        Some(true) => Ok(Classification::Decided(())),
        Some(false) => {
            let clockwise = match round_join_clockwise(center, from, to, policy) {
                Classification::Decided(clockwise) => clockwise,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            match round_join_arc(from, to, center, clockwise) {
                Classification::Decided(arc) => {
                    joined.push(Segment2::Arc(arc));
                    Ok(Classification::Decided(()))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn append_bevel_join_if_needed(
    joined: &mut Vec<Segment2>,
    from: &Point2,
    to: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    match is_zero(&from.distance_squared(to), policy) {
        Some(true) => Ok(Classification::Decided(())),
        Some(false) => {
            joined.push(Segment2::Line(LineSeg2::try_new(from.clone(), to.clone())?));
            Ok(Classification::Decided(()))
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn round_join_arc(
    from: &Point2,
    to: &Point2,
    center: &Point2,
    clockwise: bool,
) -> Classification<CircularArc2> {
    match CircularArc2::try_from_center(from.clone(), to.clone(), center.clone(), clockwise) {
        Ok(arc) => Classification::Decided(arc),
        // A round join is only valid when both offset endpoints are certified
        // to lie on the circle around the source vertex. If exact radii differ,
        // the primitive join stage has reached the unsupported trim/rebuild
        // boundary required by the profile-offset construction above.
        Err(CurveError::ZeroRadiusArc | CurveError::RadiusMismatch) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn round_join_clockwise(
    center: &Point2,
    from: &Point2,
    to: &Point2,
    policy: &CurveContext,
) -> Classification<bool> {
    let from_radius = from.delta_from(center);
    let to_radius = to.delta_from(center);
    let turn = cross(&from_radius.0, &from_radius.1, &to_radius.0, &to_radius.1);

    match real_sign(&turn, policy) {
        Some(RealSign::Positive) => Classification::Decided(false),
        Some(RealSign::Negative) => Classification::Decided(true),
        Some(RealSign::Zero) => Classification::Decided(true),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn cross(ax: &Real, ay: &Real, bx: &Real, by: &Real) -> Real {
    (ax * by) - (ay * bx)
}
