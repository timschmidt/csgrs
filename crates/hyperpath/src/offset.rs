//! Exact offset candidates for path planning.
//!
//! Offsetting is a candidate-generation stage in contour-parallel CAM and PCB
//! clearance planning. CGAL's straight-skeleton and arrangement packages make
//! the same architectural split: generate offset curves, then let arrangement
//! predicates decide topology. This module starts with axis-aligned line
//! offsets over exact `Real` coordinates, following Yap's rule that the object
//! layer should preserve construction facts instead of flattening candidates
//! through primitive tolerances.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealSign};

use crate::arc::{ArcDirection, CircularArc, CircularArcError, ExplicitCircularArc};
use crate::bezier::{BezierParameter, CubicBezier, HigherOrderBezier, QuadraticBezier};
use crate::segment::{Axis, LinePathSegment};

/// Side of a directed path segment to offset toward.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OffsetSide {
    /// Offset to the left of the directed segment.
    Left,
    /// Offset to the right of the directed segment.
    Right,
}

/// One exact line-offset candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct LineOffsetCandidate {
    /// Side used to construct this candidate.
    pub side: OffsetSide,
    /// Exact offset distance.
    pub distance: Real,
    /// Offset segment with source provenance preserved.
    pub segment: LinePathSegment,
}

/// Errors while constructing an exact offset candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineOffsetError {
    /// The source segment was not certified axis-aligned.
    NotAxisAligned,
    /// The source segment direction could not be certified.
    UnknownDirection,
    /// Offset distance was structurally negative.
    NegativeDistance,
}

/// One exact circular-arc offset candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct ArcOffsetCandidate {
    /// Side used to construct this candidate.
    pub side: OffsetSide,
    /// Exact offset distance.
    pub distance: Real,
    /// Offset cardinal arc with source provenance preserved.
    pub arc: CircularArc,
}

/// One exact explicit circular-arc offset candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitArcOffsetCandidate {
    /// Side used to construct this candidate.
    pub side: OffsetSide,
    /// Exact offset distance.
    pub distance: Real,
    /// Offset explicit arc with source provenance preserved.
    pub arc: ExplicitCircularArc,
}

/// Errors while constructing an exact arc offset candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArcOffsetError {
    /// Offset distance was structurally negative.
    NegativeDistance,
    /// Inward offset would make the radius zero or negative.
    RadiusWouldCollapse,
    /// Exact endpoint radial scaling failed.
    EndpointScaleFailed,
    /// The resulting arc failed ordinary arc construction validation.
    InvalidArc(CircularArcError),
}

/// One exact Bezier offset sample candidate.
///
/// Polynomial Bezier offsets are generally not polynomial Beziers. Following
/// Yap's exact-computation boundary and the staged offset approaches discussed
/// by Tiller-Hanson, Levien, and Blend2D/Yzerman, this candidate records the
/// exact local offset facts without pretending they form a completed offset
/// curve. Arrangement and fitting code can consume the retained point,
/// hodograph, normal, and speed facts; when `hyperreal` can exactly represent
/// the unit-normal division, `offset_point` is populated as an exact witness.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierOffsetSampleCandidate {
    /// Side used to construct this local candidate.
    pub side: OffsetSide,
    /// Exact source parameter.
    pub parameter: BezierParameter,
    /// Exact offset distance.
    pub distance: Real,
    /// Exact source point.
    pub point: Point2,
    /// Exact hodograph at the source point.
    pub tangent: Point2,
    /// Exact side normal. This vector is not unit-normalized.
    pub normal: Point2,
    /// Exact squared speed, equal to `tangent . tangent`.
    pub speed_squared: Real,
    /// Exact squared offset distance.
    pub offset_distance_squared: Real,
    /// Exact unit-normal offset witness when square-root normalization succeeds.
    pub offset_point: Option<Point2>,
}

/// Errors while constructing exact Bezier offset sample candidates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierOffsetError {
    /// Offset distance was structurally negative.
    NegativeDistance,
    /// Hodograph speed was zero or could not be certified positive.
    DegenerateTangent,
}

/// Offset a certified axis-aligned line segment by an exact distance.
///
/// This does not claim that the resulting path is a valid pocket, trace, or
/// contour offset. It only constructs the local candidate exactly; arrangement,
/// winding, gouge, and clearance predicates must still certify the candidate
/// before it becomes output.
pub fn offset_axis_aligned_segment(
    segment: &LinePathSegment,
    distance: Real,
    side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<LineOffsetCandidate, LineOffsetError> {
    if distance.structural_facts().sign == Some(RealSign::Negative) {
        return Err(LineOffsetError::NegativeDistance);
    }
    let axis = segment
        .facts()
        .axis_aligned
        .ok_or(LineOffsetError::NotAxisAligned)?;
    let direction = segment_direction(segment, axis, policy)?;
    let (dx, dy) = normal_delta(axis, direction, side, distance.clone());
    let start = Point2::new(
        segment.start().x.clone() + dx.clone(),
        segment.start().y.clone() + dy.clone(),
    );
    let end = Point2::new(segment.end().x.clone() + dx, segment.end().y.clone() + dy);
    Ok(LineOffsetCandidate {
        side,
        distance,
        segment: LinePathSegment::with_provenance(start, end, segment.provenance()),
    })
}

/// Offset a cardinal circular arc by an exact distance.
///
/// For a counter-clockwise arc, the left side is outward; for a clockwise arc,
/// the right side is outward. The function only updates the radius and
/// reconstructs the same cardinal arc. It does not decide whether neighboring
/// offset pieces intersect or whether the final contour is valid; that remains
/// arrangement and predicate work.
pub fn offset_cardinal_arc(
    arc: &CircularArc,
    distance: Real,
    side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<ArcOffsetCandidate, ArcOffsetError> {
    if distance.structural_facts().sign == Some(RealSign::Negative) {
        return Err(ArcOffsetError::NegativeDistance);
    }
    let outward = matches!(
        (arc.direction(), side),
        (ArcDirection::Ccw, OffsetSide::Left) | (ArcDirection::Cw, OffsetSide::Right)
    );
    let radius = if outward {
        arc.radius().clone() + distance.clone()
    } else {
        let candidate = arc.radius().clone() - distance.clone();
        let ordering = compare_reals_with_policy(&candidate, &Real::zero(), policy).value();
        if !matches!(ordering, Some(Ordering::Greater)) {
            return Err(ArcOffsetError::RadiusWouldCollapse);
        }
        candidate
    };
    let offset_arc = CircularArc::cardinal_with_provenance(
        arc.center().clone(),
        radius,
        arc.start_cardinal(),
        arc.end_cardinal(),
        arc.direction(),
        arc.provenance(),
    )
    .map_err(ArcOffsetError::InvalidArc)?;
    Ok(ArcOffsetCandidate {
        side,
        distance,
        arc: offset_arc,
    })
}

/// Offset an explicit circular arc by an exact distance.
///
/// The new arc keeps the same center and angular endpoint directions. Endpoint
/// coordinates are rebuilt by scaling each retained radial vector by
/// `new_radius / old_radius`, then the explicit arc constructor replays the
/// exact circle equation. This is still only a local offset candidate; CGAL
/// arrangement-style intersection cleanup and Yap-style certified topology
/// decisions remain downstream work.
pub fn offset_explicit_arc(
    arc: &ExplicitCircularArc,
    distance: Real,
    side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<ExplicitArcOffsetCandidate, ArcOffsetError> {
    if distance.structural_facts().sign == Some(RealSign::Negative) {
        return Err(ArcOffsetError::NegativeDistance);
    }
    let outward = matches!(
        (arc.direction(), side),
        (ArcDirection::Ccw, OffsetSide::Left) | (ArcDirection::Cw, OffsetSide::Right)
    );
    let radius = if outward {
        arc.radius().clone() + distance.clone()
    } else {
        let candidate = arc.radius().clone() - distance.clone();
        let ordering = compare_reals_with_policy(&candidate, &Real::zero(), policy).value();
        if !matches!(ordering, Some(Ordering::Greater)) {
            return Err(ArcOffsetError::RadiusWouldCollapse);
        }
        candidate
    };
    let scale =
        (radius.clone() / arc.radius().clone()).map_err(|_| ArcOffsetError::EndpointScaleFailed)?;
    let start = scaled_radial_point(arc.center(), arc.start(), &scale);
    let end = scaled_radial_point(arc.center(), arc.end(), &scale);
    let offset_arc = ExplicitCircularArc::with_provenance(
        arc.center().clone(),
        radius,
        start,
        end,
        arc.direction(),
        arc.provenance(),
    )
    .map_err(ArcOffsetError::InvalidArc)?;
    Ok(ExplicitArcOffsetCandidate {
        side,
        distance,
        arc: offset_arc,
    })
}

/// Build an exact local offset sample for a quadratic Bezier.
pub fn offset_quadratic_bezier_sample(
    curve: &QuadraticBezier,
    parameter: BezierParameter,
    distance: Real,
    side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<BezierOffsetSampleCandidate, BezierOffsetError> {
    let point = curve.eval(parameter);
    let tangent = curve.derivative(parameter);
    bezier_offset_sample_from_parts(point, tangent, parameter, distance, side, policy)
}

/// Build an exact local offset sample for a cubic Bezier.
pub fn offset_cubic_bezier_sample(
    curve: &CubicBezier,
    parameter: BezierParameter,
    distance: Real,
    side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<BezierOffsetSampleCandidate, BezierOffsetError> {
    let point = curve.eval(parameter);
    let tangent = curve.derivative(parameter);
    bezier_offset_sample_from_parts(point, tangent, parameter, distance, side, policy)
}

/// Build an exact local offset sample for a quartic or quintic Bezier.
pub fn offset_higher_order_bezier_sample(
    curve: &HigherOrderBezier,
    parameter: BezierParameter,
    distance: Real,
    side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<BezierOffsetSampleCandidate, BezierOffsetError> {
    let point = curve.eval(parameter);
    let tangent = curve.derivative(parameter);
    bezier_offset_sample_from_parts(point, tangent, parameter, distance, side, policy)
}

fn segment_direction(
    segment: &LinePathSegment,
    axis: Axis,
    policy: PredicatePolicy,
) -> Result<Ordering, LineOffsetError> {
    let ordering = match axis {
        Axis::X => compare_reals_with_policy(&segment.start().x, &segment.end().x, policy).value(),
        Axis::Y => compare_reals_with_policy(&segment.start().y, &segment.end().y, policy).value(),
    };
    match ordering {
        Some(Ordering::Less | Ordering::Greater) => Ok(ordering.unwrap()),
        Some(Ordering::Equal) | None => Err(LineOffsetError::UnknownDirection),
    }
}

fn bezier_offset_sample_from_parts(
    point: Point2,
    tangent: Point2,
    parameter: BezierParameter,
    distance: Real,
    side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<BezierOffsetSampleCandidate, BezierOffsetError> {
    if distance.structural_facts().sign == Some(RealSign::Negative) {
        return Err(BezierOffsetError::NegativeDistance);
    }
    let speed_squared = Real::signed_product_sum(
        [true, true],
        [[&tangent.x, &tangent.x], [&tangent.y, &tangent.y]],
    );
    let ordering = compare_reals_with_policy(&speed_squared, &Real::zero(), policy).value();
    if !matches!(ordering, Some(Ordering::Greater)) {
        return Err(BezierOffsetError::DegenerateTangent);
    }
    let normal = bezier_normal(&tangent, side);
    let offset_distance_squared = distance.clone() * distance.clone();
    let offset_point = speed_squared
        .clone()
        .sqrt()
        .ok()
        .and_then(|speed| (distance.clone() / speed).ok())
        .map(|scale| {
            Point2::new(
                point.x.clone() + normal.x.clone() * scale.clone(),
                point.y.clone() + normal.y.clone() * scale,
            )
        });
    Ok(BezierOffsetSampleCandidate {
        side,
        parameter,
        distance,
        point,
        tangent,
        normal,
        speed_squared,
        offset_distance_squared,
        offset_point,
    })
}

fn bezier_normal(tangent: &Point2, side: OffsetSide) -> Point2 {
    match side {
        OffsetSide::Left => Point2::new(-tangent.y.clone(), tangent.x.clone()),
        OffsetSide::Right => Point2::new(tangent.y.clone(), -tangent.x.clone()),
    }
}

fn scaled_radial_point(center: &Point2, point: &Point2, scale: &Real) -> Point2 {
    Point2::new(
        center.x.clone() + (point.x.clone() - center.x.clone()) * scale.clone(),
        center.y.clone() + (point.y.clone() - center.y.clone()) * scale.clone(),
    )
}

fn normal_delta(axis: Axis, direction: Ordering, side: OffsetSide, distance: Real) -> (Real, Real) {
    let zero = Real::zero();
    match (axis, direction, side) {
        (Axis::X, Ordering::Less, OffsetSide::Left)
        | (Axis::X, Ordering::Greater, OffsetSide::Right) => (zero, distance),
        (Axis::X, Ordering::Less, OffsetSide::Right)
        | (Axis::X, Ordering::Greater, OffsetSide::Left) => (zero, -distance),
        (Axis::Y, Ordering::Less, OffsetSide::Left)
        | (Axis::Y, Ordering::Greater, OffsetSide::Right) => (-distance, zero),
        (Axis::Y, Ordering::Less, OffsetSide::Right)
        | (Axis::Y, Ordering::Greater, OffsetSide::Left) => (distance, zero),
        (_, Ordering::Equal, _) => (zero, distance),
    }
}
