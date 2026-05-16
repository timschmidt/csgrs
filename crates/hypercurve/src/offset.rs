//! Primitive parallel offsets for line and circular-arc segments.

use hyperlattice::{Backend, Scalar, ScalarSign};

use crate::classify::{is_zero, scalar_sign};
use crate::contour::Contour2;
use crate::curve_string::CurveString2;
use crate::segment::{CircularArc2, LineSeg2, Segment2};
use crate::{Classification, CurveError, CurvePolicy, CurveResult, Point2, UncertaintyReason};

/// Endpoint cap style for checked open curve-string outlines.
///
/// The cap is applied after the source curve string has been offset on both
/// sides. This enum describes only the endpoint construction; joins along the
/// left and right traces still use the primitive offset and line/round-join
/// machinery documented on [`CurveString2::offset_left_with_line_joins`].
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

impl<B: Backend> LineSeg2<B> {
    /// Returns the constant-distance segment on this segment's left side.
    ///
    /// The offset direction is the normalized left normal `(-dy, dx) / length`.
    /// This is the primitive line-profile case used by profile offset
    /// algorithms; higher-level curve-string offsetting must still add joins,
    /// trim self-intersections, and rebuild topology. See Tiller and Hanson,
    /// "Offsets of Two-Dimensional Profiles" (1984), for the line/arc
    /// primitive plus trim-and-join framing used by many CAD offset pipelines.
    pub fn offset_left(&self, distance: Scalar<B>) -> CurveResult<Self> {
        let length = self.length_squared().sqrt()?;
        let (dx, dy) = self.delta();
        let normal_x = ((-dy) / &length)?;
        let normal_y = (dx / &length)?;
        let offset_x = &normal_x * &distance;
        let offset_y = &normal_y * &distance;

        Self::try_new(
            self.start().translated(offset_x.clone(), offset_y.clone()),
            self.end().translated(offset_x, offset_y),
        )
    }
}

impl<B: Backend> CircularArc2<B> {
    /// Returns the constant-distance arc on this arc's left side.
    ///
    /// Counter-clockwise arcs have their left normal on the circle interior, so
    /// a positive offset decreases radius. Clockwise arcs have their left normal
    /// on the exterior, so a positive offset increases radius. Radius collapse
    /// and radius sign reversal are returned as explicit uncertainty because
    /// the primitive arc no longer has a valid circular-arc image at that
    /// distance. Tiller and Hanson, "Offsets of Two-Dimensional Profiles"
    /// (1984), describe this concentric-arc primitive as one step in a complete
    /// profile offset pipeline.
    pub fn offset_left(
        &self,
        distance: Scalar<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let radius = self.radius_squared().sqrt()?;
        let offset_radius = if self.is_clockwise() {
            &radius + &distance
        } else {
            &radius - &distance
        };

        match scalar_sign(&offset_radius, policy) {
            Some(ScalarSign::Positive) => {}
            Some(ScalarSign::Zero | ScalarSign::Negative) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::ScalarSign)),
        }

        let radius_scale = (offset_radius / &radius)?;
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

impl<B: Backend> Segment2<B> {
    /// Returns this segment's left-side primitive offset.
    ///
    /// Lines always produce a translated line. Arcs produce a concentric arc
    /// when the requested distance leaves a positive radius; radius collapse or
    /// reversal is reported as uncertainty instead of fabricating degenerate
    /// topology.
    pub fn offset_left(
        &self,
        distance: Scalar<B>,
        policy: &CurvePolicy,
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

impl<B: Backend> CurveString2<B> {
    /// Returns a left offset of this open curve string with straight-line joins.
    ///
    /// This is a raw offset-construction layer, not a full offset engine. Each
    /// source segment is first replaced by its primitive parallel offset. Adjacent
    /// offset lines are mitered by intersecting their supporting lines; joins
    /// that cannot be mitered are connected by a circular arc centered at the
    /// original shared vertex. A complete profile offset pipeline still has to
    /// classify join style, trim self-intersections, and cap open endpoints.
    /// Tiller and Hanson, "Offsets of Two-Dimensional Profiles" (1984),
    /// describe this staged primitive, join, and trim structure for
    /// two-dimensional profile offsets.
    pub fn offset_left_with_line_joins(
        &self,
        distance: Scalar<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if is_zero(&distance, policy) == Some(true) {
            return Ok(Classification::Decided(self.clone()));
        }

        let offsets = match offset_segments_left(self.segments(), &distance, policy)? {
            Classification::Decided(offsets) => offsets,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let joined = match joined_offset_segments(self.segments(), &offsets, false, policy)? {
            Classification::Decided(joined) => joined,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(CurveString2::new_unchecked(joined)))
    }

    /// Returns a raw joined left offset, rejecting self-contacting output.
    ///
    /// This method does not trim self-intersections or cap open endpoints. It
    /// runs the joined open offset construction and then classifies the result
    /// with [`CurveString2::has_self_contacts`]. A detected self contact is
    /// reported as explicit uncertainty so callers can choose a future trimming
    /// path instead of consuming invalid raw linework.
    pub fn offset_left_checked(
        &self,
        distance: Scalar<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let offset = match self.offset_left_with_line_joins(distance, policy)? {
            Classification::Decided(offset) => offset,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        match offset.has_self_contacts(policy)? {
            Classification::Decided(false) => Ok(Classification::Decided(offset)),
            Classification::Decided(true) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Builds a checked closed outline around this open curve string.
    ///
    /// The outline follows the left joined offset, applies the selected
    /// [`OffsetCap`] at the end point, returns along the reversed right joined
    /// offset, and applies the matching cap at the start point. The `distance`
    /// is the half-width of the outline and must be strictly positive under
    /// the active policy. As with [`CurveString2::offset_left_checked`], this
    /// is still the raw offset-construction stage described by Tiller and
    /// Hanson, "Offsets of Two-Dimensional Profiles" (1984): self-contacting
    /// input or output is rejected as explicit uncertainty until the
    /// trim/rebuild stage exists.
    pub fn offset_outline(
        &self,
        distance: Scalar<B>,
        cap: OffsetCap,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2<B>>> {
        match cap {
            OffsetCap::Round => self.offset_outline_round_caps(distance, policy),
            OffsetCap::Butt => self.offset_outline_butt_caps(distance, policy),
            OffsetCap::Square => self.offset_outline_square_caps(distance, policy),
        }
    }

    /// Builds a checked closed outline around this open curve string.
    ///
    /// The outline follows the left joined offset, adds a round cap at the end
    /// point, returns along the reversed right joined offset, and adds a round
    /// cap at the start point. The `distance` is the half-width of the outline
    /// and must be strictly positive under the active policy. As with
    /// [`CurveString2::offset_left_checked`], this is still a raw offset
    /// construction: if the input or resulting closed outline self-contacts,
    /// the method returns explicit uncertainty instead of trimming. Tiller and
    /// Hanson, "Offsets of Two-Dimensional Profiles" (1984), describe this
    /// primitive offset, cap, and trim decomposition for open profile offsets.
    pub fn offset_outline_round_caps(
        &self,
        distance: Scalar<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2<B>>> {
        let offsets = match checked_outline_offsets(self, distance, policy)? {
            Classification::Decided(offsets) => offsets,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let OutlineOffsets {
            left,
            right,
            start_center,
            end_center,
            left_start,
            left_end,
            right_start,
            right_end,
        } = offsets;

        let mut segments = Vec::with_capacity(left.len() + right.len() + 2);
        segments.extend(left.into_segments());
        match round_cap_arc(&left_end, &right_end, &end_center, policy)? {
            Classification::Decided(cap) => segments.push(Segment2::Arc(cap)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        segments.extend(reversed_segments(right.into_segments()));
        match round_cap_arc(&right_start, &left_start, &start_center, policy)? {
            Classification::Decided(cap) => segments.push(Segment2::Arc(cap)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        checked_outline_contour(segments, policy)
    }

    /// Builds a checked closed outline around this open curve string.
    ///
    /// This variant connects the left and right offset traces with straight
    /// endpoint caps. Those cap lines are the radial/perpendicular endpoint
    /// connectors in the same primitive-offset, cap, and trim decomposition
    /// used for open profiles by Tiller and Hanson, "Offsets of
    /// Two-Dimensional Profiles" (1984). The distance is the half-width and
    /// must be strictly positive. As with round caps, this constructor rejects
    /// self-contacting input or output instead of trimming the raw outline.
    pub fn offset_outline_butt_caps(
        &self,
        distance: Scalar<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2<B>>> {
        let offsets = match checked_outline_offsets(self, distance, policy)? {
            Classification::Decided(offsets) => offsets,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let OutlineOffsets {
            left,
            right,
            left_start,
            left_end,
            right_start,
            right_end,
            ..
        } = offsets;

        let mut segments = Vec::with_capacity(left.len() + right.len() + 2);
        segments.extend(left.into_segments());
        match cap_line(&left_end, &right_end)? {
            Classification::Decided(cap) => segments.push(Segment2::Line(cap)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        segments.extend(reversed_segments(right.into_segments()));
        match cap_line(&right_start, &left_start)? {
            Classification::Decided(cap) => segments.push(Segment2::Line(cap)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        checked_outline_contour(segments, policy)
    }

    /// Builds a checked closed outline with square endpoint caps.
    ///
    /// Square caps extend both offset traces by one half-width along the source
    /// endpoint tangent before connecting them with a straight cap line. For
    /// line endpoints this can be folded into the endpoint offset segment; for
    /// arc endpoints it becomes an explicit tangent extension line so the
    /// circular offset arc remains exact. This is still the primitive
    /// offset/cap construction stage described by Tiller and Hanson, "Offsets
    /// of Two-Dimensional Profiles" (1984): self-contacting input or output is
    /// rejected as uncertainty until the trim/rebuild stage exists.
    pub fn offset_outline_square_caps(
        &self,
        distance: Scalar<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2<B>>> {
        let offsets = match checked_outline_offsets(self, distance.clone(), policy)? {
            Classification::Decided(offsets) => offsets,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let OutlineOffsets {
            left,
            right,
            left_start,
            left_end,
            right_start,
            right_end,
            ..
        } = offsets;

        let start_tangent = unit_tangent_at_segment_start(
            self.segments()
                .first()
                .ok_or(CurveError::EmptyCurveString)?,
        )?;
        let end_tangent = unit_tangent_at_segment_end(
            self.segments().last().ok_or(CurveError::EmptyCurveString)?,
        )?;
        let start_dx = &start_tangent.0 * &distance;
        let start_dy = &start_tangent.1 * &distance;
        let end_dx = &end_tangent.0 * &distance;
        let end_dy = &end_tangent.1 * &distance;

        let left_start_square = left_start.translated(-start_dx.clone(), -start_dy.clone());
        let right_start_square = right_start.translated(-start_dx, -start_dy);
        let left_end_square = left_end.translated(end_dx.clone(), end_dy.clone());
        let right_end_square = right_end.translated(end_dx, end_dy);

        let left = match extend_square_cap_trace(
            left.into_segments(),
            left_start_square.clone(),
            left_end_square.clone(),
        )? {
            Classification::Decided(left) => left,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let right = match extend_square_cap_trace(
            right.into_segments(),
            right_start_square.clone(),
            right_end_square.clone(),
        )? {
            Classification::Decided(right) => right,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        let mut segments = Vec::with_capacity(left.len() + right.len() + 2);
        segments.extend(left);
        match cap_line(&left_end_square, &right_end_square)? {
            Classification::Decided(cap) => segments.push(Segment2::Line(cap)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        segments.extend(reversed_segments(right));
        match cap_line(&right_start_square, &left_start_square)? {
            Classification::Decided(cap) => segments.push(Segment2::Line(cap)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        checked_outline_contour(segments, policy)
    }
}

impl<B: Backend> Contour2<B> {
    /// Returns a left offset of this closed contour with straight-line joins.
    ///
    /// Line-line corners are mitered at the exact supporting-line intersection
    /// whenever that relation can be classified. Joins that cannot be mitered
    /// are connected by a circular arc centered at the original shared vertex.
    /// The returned contour is checked for closure, but this method deliberately
    /// does not trim self-intersections or resolve collapsed regions; those
    /// operations belong to the later full offset pipeline described by Tiller
    /// and Hanson, "Offsets of Two-Dimensional Profiles" (1984).
    pub fn offset_left_with_line_joins(
        &self,
        distance: Scalar<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if is_zero(&distance, policy) == Some(true) {
            return Ok(Classification::Decided(self.clone()));
        }

        let offsets = match offset_segments_left(self.segments(), &distance, policy)? {
            Classification::Decided(offsets) => offsets,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let joined = match joined_offset_segments(self.segments(), &offsets, true, policy)? {
            Classification::Decided(joined) => joined,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(Contour2::new_unchecked(
            CurveString2::new_unchecked(joined),
            self.fill_rule(),
        )))
    }

    /// Returns a raw joined left offset, rejecting self-contacting output.
    ///
    /// This method does not trim self-intersections. It runs the joined offset
    /// construction and then classifies the result with
    /// [`Contour2::has_self_contacts`]. A detected self contact is reported as
    /// explicit uncertainty so callers do not mistake an untrimmed raw offset
    /// for a regularized contour.
    pub fn offset_left_checked(
        &self,
        distance: Scalar<B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let offset = match self.offset_left_with_line_joins(distance, policy)? {
            Classification::Decided(offset) => offset,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        match offset.has_self_contacts(policy)? {
            Classification::Decided(false) => Ok(Classification::Decided(offset)),
            Classification::Decided(true) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }
}

fn scale_from_center<B: Backend>(
    point: &Point2<B>,
    center: &Point2<B>,
    scale: &Scalar<B>,
) -> Point2<B> {
    let radius = point.delta_from(center);
    Point2::new(
        center.x() + (&radius.0 * scale),
        center.y() + (&radius.1 * scale),
    )
}

struct OutlineOffsets<B: Backend> {
    left: CurveString2<B>,
    right: CurveString2<B>,
    start_center: Point2<B>,
    end_center: Point2<B>,
    left_start: Point2<B>,
    left_end: Point2<B>,
    right_start: Point2<B>,
    right_end: Point2<B>,
}

// Shared setup for open-profile outlines. The public cap variants differ only
// in how they connect the two offset traces; both must enforce the same
// positive-distance and no-self-contact preconditions before exposing a closed
// contour to callers.
fn checked_outline_offsets<B: Backend>(
    source: &CurveString2<B>,
    distance: Scalar<B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<OutlineOffsets<B>>> {
    match scalar_sign(&distance, policy) {
        Some(ScalarSign::Positive) => {}
        Some(ScalarSign::Zero | ScalarSign::Negative) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::ScalarSign)),
    }

    match source.has_self_contacts(policy)? {
        Classification::Decided(false) => {}
        Classification::Decided(true) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let left = match source.offset_left_with_line_joins(distance.clone(), policy)? {
        Classification::Decided(left) => left,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let right = match source.offset_left_with_line_joins(-distance, policy)? {
        Classification::Decided(right) => right,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    Ok(Classification::Decided(OutlineOffsets {
        start_center: source.start().ok_or(CurveError::EmptyCurveString)?.clone(),
        end_center: source.end().ok_or(CurveError::EmptyCurveString)?.clone(),
        left_start: left.start().ok_or(CurveError::EmptyCurveString)?.clone(),
        left_end: left.end().ok_or(CurveError::EmptyCurveString)?.clone(),
        right_start: right.start().ok_or(CurveError::EmptyCurveString)?.clone(),
        right_end: right.end().ok_or(CurveError::EmptyCurveString)?.clone(),
        left,
        right,
    }))
}

// Contour closure and self-contact checks are deliberately centralized so every
// open-outline cap style preserves the same "raw construction, no trimming"
// contract described by Tiller and Hanson (1984).
fn checked_outline_contour<B: Backend>(
    segments: Vec<Segment2<B>>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Contour2<B>>> {
    let outline = match Contour2::try_new(segments) {
        Ok(outline) => outline,
        Err(CurveError::DisconnectedCurveString) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Err(CurveError::AmbiguousCurveStringConnection) => {
            return Ok(Classification::Uncertain(UncertaintyReason::ScalarSign));
        }
        Err(error) => return Err(error),
    };
    match outline.has_self_contacts(policy)? {
        Classification::Decided(false) => Ok(Classification::Decided(outline)),
        Classification::Decided(true) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn extend_square_cap_trace<B: Backend>(
    mut segments: Vec<Segment2<B>>,
    extended_start: Point2<B>,
    extended_end: Point2<B>,
) -> CurveResult<Classification<Vec<Segment2<B>>>> {
    if segments.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }

    let original_start = segments[0].start().clone();
    match &segments[0] {
        Segment2::Line(line) => {
            segments[0] = match cap_line(&extended_start, line.end())? {
                Classification::Decided(line) => Segment2::Line(line),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        }
        Segment2::Arc(_) => match cap_line(&extended_start, &original_start)? {
            Classification::Decided(line) => segments.insert(0, Segment2::Line(line)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        },
    }

    let last_index = segments.len() - 1;
    let original_end = segments[last_index].end().clone();
    match &segments[last_index] {
        Segment2::Line(line) => {
            segments[last_index] = match cap_line(line.start(), &extended_end)? {
                Classification::Decided(line) => Segment2::Line(line),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        }
        Segment2::Arc(_) => match cap_line(&original_end, &extended_end)? {
            Classification::Decided(line) => segments.push(Segment2::Line(line)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        },
    }

    Ok(Classification::Decided(segments))
}

fn unit_tangent_at_segment_start<B: Backend>(
    segment: &Segment2<B>,
) -> CurveResult<(Scalar<B>, Scalar<B>)> {
    match segment {
        Segment2::Line(line) => unit_tangent_for_line(line),
        Segment2::Arc(arc) => unit_tangent_for_arc_at_point(arc, arc.start()),
    }
}

fn unit_tangent_at_segment_end<B: Backend>(
    segment: &Segment2<B>,
) -> CurveResult<(Scalar<B>, Scalar<B>)> {
    match segment {
        Segment2::Line(line) => unit_tangent_for_line(line),
        Segment2::Arc(arc) => unit_tangent_for_arc_at_point(arc, arc.end()),
    }
}

fn unit_tangent_for_line<B: Backend>(line: &LineSeg2<B>) -> CurveResult<(Scalar<B>, Scalar<B>)> {
    let length = line.length_squared().sqrt()?;
    let (dx, dy) = line.delta();
    Ok(((dx / &length)?, (dy / &length)?))
}

fn unit_tangent_for_arc_at_point<B: Backend>(
    arc: &CircularArc2<B>,
    point: &Point2<B>,
) -> CurveResult<(Scalar<B>, Scalar<B>)> {
    let radius = arc.radius_squared().sqrt()?;
    let (rx, ry) = point.delta_from(arc.center());
    if arc.is_clockwise() {
        Ok(((ry / &radius)?, ((-rx) / &radius)?))
    } else {
        Ok(((-ry / &radius)?, (rx / &radius)?))
    }
}

fn offset_segments_left<B: Backend>(
    segments: &[Segment2<B>],
    distance: &Scalar<B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Segment2<B>>>> {
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
enum OffsetJoin<B: Backend> {
    Miter(Point2<B>),
    Round { center: Point2<B> },
}

fn joined_offset_segments<B: Backend>(
    source: &[Segment2<B>],
    offsets: &[Segment2<B>],
    closed: bool,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Segment2<B>>>> {
    if offsets.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }
    if source.len() != offsets.len() {
        return Err(CurveError::Topology(
            "source and offset segment counts differ".into(),
        ));
    }

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
            policy,
        )? {
            Classification::Decided(join) => joins.push(join),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    let mut joined = Vec::with_capacity(offsets.len() + join_count);
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
        joined.push(adjusted);

        if let Some(OffsetJoin::Round { center }) = joins.get(index) {
            let from = joined
                .last()
                .expect("current adjusted offset segment was just pushed")
                .end()
                .clone();
            let to = offsets[(index + 1) % offsets.len()].start().clone();
            match append_round_join_if_needed(&mut joined, &from, &to, center, policy)? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
    }

    Ok(Classification::Decided(joined))
}

fn classify_offset_join<B: Backend>(
    source_previous: &Segment2<B>,
    source_next: &Segment2<B>,
    offset_previous: &Segment2<B>,
    offset_next: &Segment2<B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<OffsetJoin<B>>> {
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

fn round_join<B: Backend>(previous: &Segment2<B>, next: &Segment2<B>) -> OffsetJoin<B> {
    let _ = next;
    OffsetJoin::Round {
        center: previous.end().clone(),
    }
}

fn line_support_intersection<B: Backend>(
    previous: &LineSeg2<B>,
    next: &LineSeg2<B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<Point2<B>>>> {
    let (rx, ry) = previous.delta();
    let (sx, sy) = next.delta();
    let denominator = cross(&rx, &ry, &sx, &sy);

    match scalar_sign(&denominator, policy) {
        Some(ScalarSign::Zero) => Ok(Classification::Decided(None)),
        Some(ScalarSign::Positive | ScalarSign::Negative) => {
            let qmp = next.start().delta_from(previous.start());
            let numerator = cross(&qmp.0, &qmp.1, &sx, &sy);
            let t = (numerator / &denominator)?;
            Ok(Classification::Decided(Some(previous.point_at(t))))
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::ScalarSign)),
    }
}

fn start_miter_for_segment<B: Backend>(
    index: usize,
    segment_count: usize,
    closed: bool,
    joins: &[OffsetJoin<B>],
) -> Option<Point2<B>> {
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

fn end_miter_for_segment<B: Backend>(index: usize, joins: &[OffsetJoin<B>]) -> Option<Point2<B>> {
    match joins.get(index) {
        Some(OffsetJoin::Miter(point)) => Some(point.clone()),
        _ => None,
    }
}

fn adjust_offset_segment<B: Backend>(
    segment: &Segment2<B>,
    start_override: Option<&Point2<B>>,
    end_override: Option<&Point2<B>>,
) -> CurveResult<Classification<Segment2<B>>> {
    match segment {
        Segment2::Line(line) => {
            let start = start_override
                .cloned()
                .unwrap_or_else(|| line.start().clone());
            let end = end_override.cloned().unwrap_or_else(|| line.end().clone());
            match LineSeg2::try_new(start, end) {
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

fn append_round_join_if_needed<B: Backend>(
    joined: &mut Vec<Segment2<B>>,
    from: &Point2<B>,
    to: &Point2<B>,
    center: &Point2<B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    let distance = from.distance_squared(to);
    match is_zero(&distance, policy) {
        Some(true) => Ok(Classification::Decided(())),
        Some(false) => {
            let clockwise = match round_join_clockwise(center, from, to, policy) {
                Classification::Decided(clockwise) => clockwise,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            joined.push(Segment2::Arc(CircularArc2::try_from_center(
                from.clone(),
                to.clone(),
                center.clone(),
                clockwise,
            )?));
            Ok(Classification::Decided(()))
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::ScalarSign)),
    }
}

fn round_cap_arc<B: Backend>(
    from: &Point2<B>,
    to: &Point2<B>,
    center: &Point2<B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<CircularArc2<B>>> {
    match is_zero(&from.distance_squared(to), policy) {
        Some(true) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        Some(false) => {
            let clockwise = match round_join_clockwise(center, from, to, policy) {
                Classification::Decided(clockwise) => clockwise,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            CircularArc2::try_from_center(from.clone(), to.clone(), center.clone(), clockwise)
                .map(Classification::Decided)
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::ScalarSign)),
    }
}

fn cap_line<B: Backend>(
    from: &Point2<B>,
    to: &Point2<B>,
) -> CurveResult<Classification<LineSeg2<B>>> {
    match LineSeg2::try_new(from.clone(), to.clone()) {
        Ok(line) => Ok(Classification::Decided(line)),
        Err(CurveError::ZeroLengthLine) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Err(error) => Err(error),
    }
}

fn reversed_segments<B: Backend>(segments: Vec<Segment2<B>>) -> impl Iterator<Item = Segment2<B>> {
    segments.into_iter().rev().map(|segment| segment.reversed())
}

fn round_join_clockwise<B: Backend>(
    center: &Point2<B>,
    from: &Point2<B>,
    to: &Point2<B>,
    policy: &CurvePolicy,
) -> Classification<bool> {
    let from_radius = from.delta_from(center);
    let to_radius = to.delta_from(center);
    let turn = cross(&from_radius.0, &from_radius.1, &to_radius.0, &to_radius.1);

    match scalar_sign(&turn, policy) {
        Some(ScalarSign::Positive) => Classification::Decided(false),
        Some(ScalarSign::Negative) => Classification::Decided(true),
        Some(ScalarSign::Zero) => Classification::Decided(true),
        None => Classification::Uncertain(UncertaintyReason::ScalarSign),
    }
}

fn cross<B: Backend>(ax: &Scalar<B>, ay: &Scalar<B>, bx: &Scalar<B>, by: &Scalar<B>) -> Scalar<B> {
    (ax * by) - (ay * bx)
}
