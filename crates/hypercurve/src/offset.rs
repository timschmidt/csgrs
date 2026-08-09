//! Primitive parallel offsets for line and circular-arc segments.
//!
//! Offsetting is split into primitive parallel curves, joins/caps, and later
//! trimming/rebuild work. Checked offsets reject raw self-intersections because
//! plane offsets may
//! form cusps and extraneous loops that require trimming.

use hyperreal::{Real, RealSign};
use std::cmp::Ordering;

use crate::classify::{classify_oriented_line, compare_reals, is_zero, real_sign};
use crate::contour::{Contour2, FillRule};
use crate::curve_string::CurveString2;
use crate::segment::{CircularArc2, LineSeg2, Segment2};
use crate::{
    Classification, CurveContext, CurveError, CurveFamily2, CurveOperation2, CurveRegion2,
    CurveResult, ExactCurveError, ExactCurveResult, LineSide, Point2, UncertaintyReason,
};

fn exact_offset_error(cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(CurveOperation2::Offset, CurveFamily2::Line, cause)
}

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

    /// Returns a raw joined left offset, rejecting self-contacting output.
    ///
    /// This method does not trim self-intersections or cap open endpoints. It
    /// runs the joined open offset construction and then classifies the result
    /// with [`CurveString2::has_self_contacts`]. A detected self contact is
    /// reported as explicit uncertainty so callers can choose a future trimming
    /// path instead of consuming invalid raw linework. Such self-intersections
    /// and extraneous loops must be trimmed before the curve can represent the
    /// intended profile.
    pub fn offset_left_checked(
        &self,
        distance: Real,
        policy: &CurveContext,
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
    /// is still the raw offset-construction stage described by profile-offset construction: self-contacting
    /// input or output is rejected as explicit uncertainty until the
    /// trim/rebuild stage exists.
    pub fn offset_outline(
        &self,
        distance: Real,
        cap: OffsetCap,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Contour2>> {
        checked_outline(self, distance, cap, policy)
    }

    /// Builds a checked closed outline around this open curve string.
    ///
    /// The outline follows the left joined offset, adds a round cap at the end
    /// point, returns along the reversed right joined offset, and adds a round
    /// cap at the start point. The `distance` is the half-width of the outline
    /// and must be strictly positive under the active policy. As with
    /// [`CurveString2::offset_left_checked`], this is still a raw offset
    /// construction: if the input or resulting closed outline self-contacts,
    /// the method returns explicit uncertainty instead of trimming.
    pub fn offset_outline_round_caps(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Contour2>> {
        self.offset_outline(distance, OffsetCap::Round, policy)
    }

    /// Builds a checked closed outline around this open curve string.
    ///
    /// This variant connects the left and right offset traces with straight
    /// endpoint caps. Those cap lines are the radial/perpendicular endpoint
    /// connectors in the same primitive-offset, cap, and trim decomposition
    /// used for open profiles by profile-offset construction. The distance is the half-width and
    /// must be strictly positive. As with round caps, this constructor rejects
    /// self-contacting input or output instead of trimming the raw outline.
    pub fn offset_outline_butt_caps(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Contour2>> {
        self.offset_outline(distance, OffsetCap::Butt, policy)
    }

    /// Builds a checked closed outline with square endpoint caps.
    ///
    /// Square caps extend both offset traces by one half-width along the source
    /// endpoint tangent before connecting them with a straight cap line. For
    /// line endpoints this can be folded into the endpoint offset segment; for
    /// arc endpoints it becomes an explicit tangent extension line so the
    /// circular offset arc remains exact. This is still the primitive
    /// offset/cap construction stage described by profile-offset construction: self-contacting input or output is
    /// rejected as uncertainty until the trim/rebuild stage exists.
    pub fn offset_outline_square_caps(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Contour2>> {
        self.offset_outline(distance, OffsetCap::Square, policy)
    }
}

impl Contour2 {
    /// Computes an exact miter erosion of a simple axis-aligned line contour.
    ///
    /// The source vertex coordinates and their `distance` translations induce
    /// a finite rectangular arrangement. On every open cell, both source
    /// containment and minimum L-infinity distance to the orthogonal boundary
    /// are constant predicates. Retaining exactly the cells that are inside and
    /// at least the erosion radius from every boundary segment handles neck
    /// collapse and component splitting without a medial-axis approximation.
    pub(crate) fn offset_left_orthogonal_line_erosion(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<CurveRegion2>> {
        let Some(area) = self.signed_area().map_err(exact_offset_error)? else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let area_sign = match real_sign(&area, policy) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let distance_sign = match real_sign(&distance, policy) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => {
                return CurveRegion2::try_from_native_contours_raw(
                    vec![self.clone()],
                    Vec::new(),
                    policy,
                )
                .map(Classification::Decided);
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        if area_sign != distance_sign {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let radius = match distance_sign {
            RealSign::Positive => distance,
            RealSign::Negative => -distance,
            RealSign::Zero => unreachable!("zero distance returned above"),
        };

        let mut source_x = Vec::with_capacity(self.len());
        let mut source_y = Vec::with_capacity(self.len());
        for segment in self.segments() {
            let Segment2::Line(line) = segment else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let (dx, dy) = line.delta();
            match (is_zero(&dx, policy), is_zero(&dy, policy)) {
                (Some(true), Some(false)) | (Some(false), Some(true)) => {}
                (Some(_), Some(_)) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                _ => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            source_x.push(line.start().x().clone());
            source_y.push(line.start().y().clone());
        }
        let source_x = match sort_dedup_exact_reals(source_x, policy) {
            Classification::Decided(values) => values,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source_y = match sort_dedup_exact_reals(source_y, policy) {
            Classification::Decided(values) => values,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (Some(min_x), Some(max_x), Some(min_y), Some(max_y)) = (
            source_x.first(),
            source_x.last(),
            source_y.first(),
            source_y.last(),
        ) else {
            return Err(exact_offset_error(CurveError::EmptyCurveString));
        };
        let x_coordinates =
            match orthogonal_erosion_coordinates(&source_x, min_x, max_x, &radius, policy) {
                Classification::Decided(values) => values,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let y_coordinates =
            match orthogonal_erosion_coordinates(&source_y, min_y, max_y, &radius, policy) {
                Classification::Decided(values) => values,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };

        let half = (Real::one() / Real::from(2_u8))
            .map_err(|problem| exact_offset_error(problem.into()))?;
        let mut cells = Vec::new();
        for x_pair in x_coordinates.windows(2) {
            for y_pair in y_coordinates.windows(2) {
                let sample = Point2::new(
                    (&x_pair[0] + &x_pair[1]) * &half,
                    (&y_pair[0] + &y_pair[1]) * &half,
                );
                match self.classify_point(&sample, policy) {
                    Classification::Decided(crate::ContourPointLocation::Inside) => {}
                    Classification::Decided(
                        crate::ContourPointLocation::Outside
                        | crate::ContourPointLocation::Boundary,
                    ) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match point_is_at_least_orthogonal_boundary_distance(&sample, self, &radius, policy)
                {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                cells.push(
                    axis_aligned_offset_cell(
                        x_pair[0].clone(),
                        y_pair[0].clone(),
                        x_pair[1].clone(),
                        y_pair[1].clone(),
                    )
                    .map_err(exact_offset_error)?,
                );
            }
        }

        let mut result = CurveRegion2::empty();
        for cell in cells {
            let component =
                CurveRegion2::try_from_native_contours_raw(vec![cell], Vec::new(), policy)?;
            result = result
                .boolean_region_raw(&component, crate::BooleanOp::Union, policy)
                .map_err(|error| error.with_operation(CurveOperation2::Offset))?;
        }
        Ok(Classification::Decided(result))
    }

    /// Computes the exact erosion of a certified convex line contour.
    ///
    /// Every source supporting line is shifted by `distance`, then all pairwise
    /// shifted-line intersections are filtered against the complete set of
    /// inward half-planes. The exact convex hull of the feasible vertices is
    /// the eroded boundary. A point, segment, or infeasible intersection is an
    /// empty two-dimensional result. Non-line, non-convex, self-contacting, or
    /// predicate-undecidable sources remain explicit uncertainty so this helper
    /// cannot silently stand in for general medial-axis pruning.
    pub(crate) fn offset_left_convex_line_erosion(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        let orientation = match certified_convex_line_orientation(self, policy)? {
            Classification::Decided(Some(orientation)) => orientation,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let offset_lines = self
            .segments()
            .iter()
            .map(|segment| match segment {
                Segment2::Line(line) => line.offset_left(distance.clone()),
                Segment2::Arc(_) => unreachable!("convex line certificate excludes arcs"),
            })
            .collect::<CurveResult<Vec<_>>>()?;

        let mut feasible = Vec::new();
        for first in 0..offset_lines.len() {
            for second in first + 1..offset_lines.len() {
                let candidate = match line_support_intersection(
                    &offset_lines[first],
                    &offset_lines[second],
                    policy,
                )? {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match point_satisfies_convex_half_planes(
                    &candidate,
                    &offset_lines,
                    [first, second],
                    orientation,
                    policy,
                ) {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match push_distinct_exact_point(&mut feasible, candidate, policy) {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }

        let mut hull = match exact_convex_hull(feasible, policy) {
            Classification::Decided(hull) => hull,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if hull.len() < 3 {
            return Ok(Classification::Decided(None));
        }
        if orientation == RealSign::Negative {
            hull.reverse();
        }
        let segments = hull
            .iter()
            .zip(hull.iter().cycle().skip(1))
            .take(hull.len())
            .map(|(start, end)| LineSeg2::try_new(start.clone(), end.clone()).map(Segment2::Line))
            .collect::<CurveResult<Vec<_>>>()?;
        Self::try_new_with_fill_rule(segments, self.fill_rule())
            .map(Some)
            .map(Classification::Decided)
    }

    /// Returns a left offset of this closed contour with straight-line joins.
    ///
    /// Line-line corners are mitered at the exact supporting-line intersection
    /// whenever that relation can be classified. Joins that cannot be mitered
    /// are connected by a circular arc centered at the original shared vertex.
    /// The returned contour is checked for closure, but this method deliberately
    /// does not trim self-intersections or resolve collapsed regions; those
    /// operations belong to the later full offset pipeline described by profile-offset construction.
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
        let joined =
            match joined_offset_segments(self.segments(), &offsets, true, None, &distance, policy)?
            {
                Classification::Decided(joined) => joined,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        Ok(checked_joined_contour(joined, self.fill_rule())
            .map(|offset| offset.retain_left_offset_from(self, distance, policy)))
    }

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

    /// Returns a raw joined left offset, rejecting self-contacting output.
    ///
    /// This method does not trim self-intersections. It runs the joined offset
    /// construction and then classifies the result with
    /// [`Contour2::has_self_contacts`]. A detected self contact is reported as
    /// explicit uncertainty so callers do not mistake an untrimmed raw offset
    /// for a regularized contour. This matches the standard offset-curve
    /// treatment of self-intersections and extraneous loops as a separate
    /// trimming stage, not a property of the primitive parallel curve itself.
    pub fn offset_left_checked(
        &self,
        distance: Real,
        policy: &CurveContext,
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

fn sort_dedup_exact_reals(values: Vec<Real>, policy: &CurveContext) -> Classification<Vec<Real>> {
    let mut sorted = Vec::<Real>::new();
    for value in values {
        let mut insertion = sorted.len();
        for (index, existing) in sorted.iter().enumerate() {
            match compare_reals(&value, existing, policy) {
                Some(Ordering::Less) => {
                    insertion = index;
                    break;
                }
                Some(Ordering::Equal) => {
                    insertion = usize::MAX;
                    break;
                }
                Some(Ordering::Greater) => {}
                None => return Classification::Uncertain(UncertaintyReason::Ordering),
            }
        }
        if insertion != usize::MAX {
            sorted.insert(insertion, value);
        }
    }
    Classification::Decided(sorted)
}

fn orthogonal_erosion_coordinates(
    source: &[Real],
    minimum: &Real,
    maximum: &Real,
    radius: &Real,
    policy: &CurveContext,
) -> Classification<Vec<Real>> {
    let mut candidates = Vec::with_capacity(source.len() * 3);
    for coordinate in source {
        candidates.push(coordinate.clone());
        candidates.push(coordinate - radius);
        candidates.push(coordinate + radius);
    }
    let candidates = match sort_dedup_exact_reals(candidates, policy) {
        Classification::Decided(values) => values,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let mut retained = Vec::new();
    for candidate in candidates {
        let above_minimum = compare_reals(&candidate, minimum, policy);
        let below_maximum = compare_reals(&candidate, maximum, policy);
        match (above_minimum, below_maximum) {
            (Some(Ordering::Equal | Ordering::Greater), Some(Ordering::Equal | Ordering::Less)) => {
                retained.push(candidate);
            }
            (Some(_), Some(_)) => {}
            _ => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(retained)
}

fn point_is_at_least_orthogonal_boundary_distance(
    point: &Point2,
    contour: &Contour2,
    radius: &Real,
    policy: &CurveContext,
) -> Classification<bool> {
    for segment in contour.segments() {
        let Segment2::Line(line) = segment else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let dx =
            match distance_to_exact_interval(point.x(), line.start().x(), line.end().x(), policy) {
                Classification::Decided(distance) => distance,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
        let dy =
            match distance_to_exact_interval(point.y(), line.start().y(), line.end().y(), policy) {
                Classification::Decided(distance) => distance,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
        let distance = match compare_reals(&dx, &dy, policy) {
            Some(Ordering::Less) => dy,
            Some(Ordering::Equal | Ordering::Greater) => dx,
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        };
        match compare_reals(&distance, radius, policy) {
            Some(Ordering::Less) => return Classification::Decided(false),
            Some(Ordering::Equal | Ordering::Greater) => {}
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(true)
}

fn distance_to_exact_interval(
    value: &Real,
    first: &Real,
    second: &Real,
    policy: &CurveContext,
) -> Classification<Real> {
    let (minimum, maximum) = match compare_reals(first, second, policy) {
        Some(Ordering::Less | Ordering::Equal) => (first, second),
        Some(Ordering::Greater) => (second, first),
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    match (
        compare_reals(value, minimum, policy),
        compare_reals(value, maximum, policy),
    ) {
        (Some(Ordering::Less), Some(_)) => Classification::Decided(minimum - value),
        (Some(_), Some(Ordering::Greater)) => Classification::Decided(value - maximum),
        (Some(_), Some(_)) => Classification::Decided(Real::zero()),
        _ => Classification::Uncertain(UncertaintyReason::Ordering),
    }
}

fn axis_aligned_offset_cell(
    min_x: Real,
    min_y: Real,
    max_x: Real,
    max_y: Real,
) -> CurveResult<Contour2> {
    let lower_left = Point2::new(min_x.clone(), min_y.clone());
    let lower_right = Point2::new(max_x.clone(), min_y);
    let upper_right = Point2::new(max_x, max_y.clone());
    let upper_left = Point2::new(min_x, max_y);
    Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(lower_left.clone(), lower_right.clone())?),
        Segment2::Line(LineSeg2::try_new(lower_right, upper_right.clone())?),
        Segment2::Line(LineSeg2::try_new(upper_right, upper_left.clone())?),
        Segment2::Line(LineSeg2::try_new(upper_left, lower_left)?),
    ])
}

fn certified_convex_line_orientation(
    contour: &Contour2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<RealSign>>> {
    if contour
        .segments()
        .iter()
        .any(|segment| !matches!(segment, Segment2::Line(_)))
    {
        return Ok(Classification::Decided(None));
    }
    match contour.has_self_contacts(policy)? {
        Classification::Decided(false) => {}
        Classification::Decided(true) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let Some(area) = contour.signed_area()? else {
        return Ok(Classification::Decided(None));
    };
    let orientation = match real_sign(&area, policy) {
        Some(RealSign::Positive) => RealSign::Positive,
        Some(RealSign::Negative) => RealSign::Negative,
        Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    for (previous, next) in contour
        .segments()
        .iter()
        .zip(contour.segments().iter().cycle().skip(1))
        .take(contour.segments().len())
    {
        let (Segment2::Line(previous), Segment2::Line(next)) = (previous, next) else {
            unreachable!("line-family inventory was certified above");
        };
        let (previous_x, previous_y) = previous.delta();
        let (next_x, next_y) = next.delta();
        match real_sign(&cross(&previous_x, &previous_y, &next_x, &next_y), policy) {
            Some(RealSign::Zero) => {}
            Some(turn) if turn == orientation => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(None));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    Ok(Classification::Decided(Some(orientation)))
}

fn point_satisfies_convex_half_planes(
    point: &Point2,
    lines: &[LineSeg2],
    supporting_lines: [usize; 2],
    orientation: RealSign,
    policy: &CurveContext,
) -> Classification<bool> {
    for (index, line) in lines.iter().enumerate() {
        // The candidate was constructed as the exact intersection of these
        // two supports. Re-evaluating their determinants can obscure that
        // construction behind an algebraically equivalent radical expression
        // that `Real` conservatively declines to prove zero.
        if supporting_lines.contains(&index) {
            continue;
        }
        match line.classify_point(point, policy) {
            Classification::Decided(LineSide::On) => {}
            Classification::Decided(LineSide::Left) if orientation == RealSign::Positive => {}
            Classification::Decided(LineSide::Right) if orientation == RealSign::Negative => {}
            Classification::Decided(LineSide::Left | LineSide::Right) => {
                return Classification::Decided(false);
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(true)
}

fn push_distinct_exact_point(
    points: &mut Vec<Point2>,
    candidate: Point2,
    policy: &CurveContext,
) -> Classification<()> {
    for point in points.iter() {
        match is_zero(&point.distance_squared(&candidate), policy) {
            Some(true) => return Classification::Decided(()),
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    points.push(candidate);
    Classification::Decided(())
}

fn exact_convex_hull(
    mut points: Vec<Point2>,
    policy: &CurveContext,
) -> Classification<Vec<Point2>> {
    for index in 1..points.len() {
        let mut cursor = index;
        while cursor > 0 {
            let ordering = match compare_points_lexicographically(
                &points[cursor],
                &points[cursor - 1],
                policy,
            ) {
                Some(ordering) => ordering,
                None => return Classification::Uncertain(UncertaintyReason::Ordering),
            };
            if ordering != Ordering::Less {
                break;
            }
            points.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    if points.len() < 3 {
        return Classification::Decided(points);
    }

    let mut lower = Vec::new();
    for point in &points {
        match append_convex_hull_point(&mut lower, point.clone(), policy) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    let mut upper = Vec::new();
    for point in points.iter().rev() {
        match append_convex_hull_point(&mut upper, point.clone(), policy) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    Classification::Decided(lower)
}

fn append_convex_hull_point(
    hull: &mut Vec<Point2>,
    point: Point2,
    policy: &CurveContext,
) -> Classification<()> {
    while hull.len() >= 2 {
        match classify_oriented_line(&hull[hull.len() - 2], &hull[hull.len() - 1], &point, policy) {
            Classification::Decided(LineSide::Left) => break,
            Classification::Decided(LineSide::On | LineSide::Right) => {
                hull.pop();
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    hull.push(point);
    Classification::Decided(())
}

fn compare_points_lexicographically(
    first: &Point2,
    second: &Point2,
    policy: &CurveContext,
) -> Option<Ordering> {
    match compare_reals(first.x(), second.x(), policy)? {
        Ordering::Equal => compare_reals(first.y(), second.y(), policy),
        ordering => Some(ordering),
    }
}

fn checked_outline(
    source: &CurveString2,
    distance: Real,
    cap: OffsetCap,
    policy: &CurveContext,
) -> CurveResult<Classification<Contour2>> {
    match real_sign(&distance, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
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
    let right = match source.offset_left_with_line_joins(-distance.clone(), policy)? {
        Classification::Decided(right) => right,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let offsets = OutlineOffsets {
        start_center: source.start().ok_or(CurveError::EmptyCurveString)?.clone(),
        end_center: source.end().ok_or(CurveError::EmptyCurveString)?.clone(),
        left_start: left.start().ok_or(CurveError::EmptyCurveString)?.clone(),
        left_end: left.end().ok_or(CurveError::EmptyCurveString)?.clone(),
        right_start: right.start().ok_or(CurveError::EmptyCurveString)?.clone(),
        right_end: right.end().ok_or(CurveError::EmptyCurveString)?.clone(),
        left,
        right,
    };
    let segments = match outline_segments_for_cap(source, offsets, distance, cap, policy)? {
        Classification::Decided(segments) => segments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    checked_outline_contour(segments, policy)
}

fn outline_segments_for_cap(
    source: &CurveString2,
    offsets: OutlineOffsets,
    distance: Real,
    cap: OffsetCap,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<Segment2>>> {
    match cap {
        OffsetCap::Round => outline_segments_with_round_caps(offsets, distance, policy),
        OffsetCap::Butt => outline_segments_with_butt_caps(offsets),
        OffsetCap::Square => outline_segments_with_square_caps(source, offsets, distance),
    }
}

fn outline_segments_with_round_caps(
    offsets: OutlineOffsets,
    distance: Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<Segment2>>> {
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
    let radius_squared = &distance * &distance;

    let mut segments = Vec::with_capacity(left.len() + right.len() + 2);
    segments.extend(left.into_segments());
    match round_cap_arc(&left_end, &right_end, &end_center, &radius_squared, policy)? {
        Classification::Decided(cap) => segments.push(Segment2::Arc(cap)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    segments.extend(reversed_segments(right.into_segments()));
    match round_cap_arc(
        &right_start,
        &left_start,
        &start_center,
        &radius_squared,
        policy,
    )? {
        Classification::Decided(cap) => segments.push(Segment2::Arc(cap)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    Ok(Classification::Decided(segments))
}

fn outline_segments_with_butt_caps(
    offsets: OutlineOffsets,
) -> CurveResult<Classification<Vec<Segment2>>> {
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

    Ok(Classification::Decided(segments))
}

fn outline_segments_with_square_caps(
    source: &CurveString2,
    offsets: OutlineOffsets,
    distance: Real,
) -> CurveResult<Classification<Vec<Segment2>>> {
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
        source
            .segments()
            .first()
            .ok_or(CurveError::EmptyCurveString)?,
    )?;
    let end_tangent = unit_tangent_at_segment_end(
        source
            .segments()
            .last()
            .ok_or(CurveError::EmptyCurveString)?,
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

    Ok(Classification::Decided(segments))
}

pub(crate) fn scale_from_center(point: &Point2, center: &Point2, scale: &Real) -> Point2 {
    let radius = point.delta_from(center);
    Point2::new(
        center.x() + (&radius.0 * scale),
        center.y() + (&radius.1 * scale),
    )
}

struct OutlineOffsets {
    left: CurveString2,
    right: CurveString2,
    start_center: Point2,
    end_center: Point2,
    left_start: Point2,
    left_end: Point2,
    right_start: Point2,
    right_end: Point2,
}

fn checked_outline_contour(
    segments: Vec<Segment2>,
    policy: &CurveContext,
) -> CurveResult<Classification<Contour2>> {
    let outline = match Contour2::try_new(segments) {
        Ok(outline) => outline,
        Err(CurveError::DisconnectedCurveString) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Err(CurveError::AmbiguousCurveStringConnection) => {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
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

fn extend_square_cap_trace(
    mut segments: Vec<Segment2>,
    extended_start: Point2,
    extended_end: Point2,
) -> CurveResult<Classification<Vec<Segment2>>> {
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

fn unit_tangent_at_segment_start(segment: &Segment2) -> CurveResult<(Real, Real)> {
    match segment {
        Segment2::Line(line) => unit_tangent_for_line(line),
        Segment2::Arc(arc) => unit_tangent_for_arc_at_point(arc, arc.start()),
    }
}

fn unit_tangent_at_segment_end(segment: &Segment2) -> CurveResult<(Real, Real)> {
    match segment {
        Segment2::Line(line) => unit_tangent_for_line(line),
        Segment2::Arc(arc) => unit_tangent_for_arc_at_point(arc, arc.end()),
    }
}

fn unit_tangent_for_line(line: &LineSeg2) -> CurveResult<(Real, Real)> {
    let (dx, dy) = line.delta();
    unit_direction_for_delta(&dx, &dy)
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

fn unit_tangent_for_arc_at_point(arc: &CircularArc2, point: &Point2) -> CurveResult<(Real, Real)> {
    let radius = arc.radius_squared().sqrt()?;
    let (rx, ry) = point.delta_from(arc.center());
    if arc.is_clockwise() {
        Ok(((ry / &radius)?, ((-rx) / &radius)?))
    } else {
        Ok(((-ry / &radius)?, (rx / &radius)?))
    }
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

fn line_support_intersection(
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

fn round_cap_arc(
    from: &Point2,
    to: &Point2,
    center: &Point2,
    radius_squared: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<CircularArc2>> {
    match is_zero(&from.distance_squared(to), policy) {
        Some(true) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        Some(false) => {
            let clockwise = match round_join_clockwise(center, from, to, policy) {
                Classification::Decided(clockwise) => clockwise,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            Ok(Classification::Decided(
                CircularArc2::new_with_certified_radius(
                    from.clone(),
                    to.clone(),
                    center.clone(),
                    radius_squared.clone(),
                    clockwise,
                    None,
                ),
            ))
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

fn cap_line(from: &Point2, to: &Point2) -> CurveResult<Classification<LineSeg2>> {
    match LineSeg2::try_new(from.clone(), to.clone()) {
        Ok(line) => Ok(Classification::Decided(line)),
        Err(CurveError::ZeroLengthLine) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Err(error) => Err(error),
    }
}

fn reversed_segments(segments: Vec<Segment2>) -> impl Iterator<Item = Segment2> {
    segments.into_iter().rev().map(|segment| segment.reversed())
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
