//! CSG grammar over native Hypercurve carriers.
//!
//! Filled construction returns [`CurveRegion2`]. Open geometry is returned as
//! [`CurveString2`] or [`CurvePath2`] by the APIs that create it; csgrs does not
//! combine those distinct native meanings in a public profile facade.

use crate::context::GeometryDecisions;
use crate::errors::ValidationError;
use crate::solid;
use crate::{GeometryContext, GeometryOutcome};
use hypercurve::{
    BooleanOp, Contour2, CubicBezier2, Curve2, CurveGeometry2, CurvePath2,
    CurvePathRegionTrim2, CurveRegion2, CurveString2, FinitePolyline2,
    FiniteProjectionOptions, FiniteRegionProfile2, LineSeg2, Point2, PolynomialSplineCurve2,
    QuadraticBezier2, RationalBezier2, RegionPointLocation, Segment2,
};
use hyperlattice::{Aabb, Matrix4, Point3, Real, Vector3};
use hypermesh::TriangleMesh;
use std::cell::RefCell;
use std::cmp::Ordering;

thread_local! {
    static REGION_RING_CACHE: RefCell<Vec<(Vec<[Real; 2]>, CurveRegion2)>> =
        const { RefCell::new(Vec::new()) };
}

fn positive_with(
    value: &Real,
    name: &'static str,
    decisions: &GeometryDecisions,
) -> Result<(), ValidationError> {
    match decisions.sign(value, name)? {
        hyperreal::RealSign::Positive => Ok(()),
        _ => Err(ValidationError::NonPositiveParameter { name }),
    }
}

fn nonnegative_with(
    value: &Real,
    name: &'static str,
    decisions: &GeometryDecisions,
) -> Result<hyperreal::RealSign, ValidationError> {
    match decisions.sign(value, name)? {
        hyperreal::RealSign::Negative => Err(ValidationError::NegativeParameter { name }),
        sign => Ok(sign),
    }
}

fn compare_with(
    left: &Real,
    right: &Real,
    decisions: &GeometryDecisions,
) -> Result<Ordering, ValidationError> {
    decisions.decide(
        hyperlimit::compare_reals(left, right, decisions.predicate_policy()),
        "curve dimensions",
    )
}

fn ring_with(
    points: &[[Real; 2]],
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    if let Some(region) = REGION_RING_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .rev()
            .find(|(key, _)| key == points)
            .map(|(_, region)| region.clone())
    }) {
        return Ok(region);
    }
    let region = decisions.exact_curve(|| -> Result<_, ValidationError> {
        let contour = Contour2::from_real_ring(points)?;
        Ok(CurveRegion2::try_from_native_material_contours(vec![
            contour,
        ])?)
    })?;
    // Only successful strict proofs enter this thread-local, 64-entry FIFO.
    // Owned exact coordinate keys and cloned regions preserve input identity.
    // Approximate requests may reuse a strict proof, but never populate it.
    if decisions.predicate_policy() == hyperlimit::PredicatePolicy::STRICT {
        REGION_RING_CACHE.with_borrow_mut(|entries| {
            if entries.len() == 64 {
                entries.remove(0);
            }
            entries.push((points.to_vec(), region.clone()));
        });
    }
    Ok(region)
}

fn rectangle_with(
    width: Real,
    height: Real,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    let width_sign = nonnegative_with(&width, "width", decisions)?;
    let height_sign = nonnegative_with(&height, "height", decisions)?;
    if width_sign == hyperreal::RealSign::Zero || height_sign == hyperreal::RealSign::Zero {
        return Ok(empty());
    }
    ring_with(
        &[
            [Real::zero(), Real::zero()],
            [width.clone(), Real::zero()],
            [width, height.clone()],
            [Real::zero(), height],
        ],
        decisions,
    )
}

fn ellipse_points_with(
    radius_x: Real,
    radius_y: Real,
    segments: usize,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let fraction = (Real::from(index as u64) / &denominator)?;
        let angle = Real::tau() * fraction;
        points.push([
            radius_x.clone() * angle.clone().cos(),
            radius_y.clone() * angle.sin(),
        ]);
    }
    ring_with(&points, decisions)
}

fn circle_with(
    radius: Real,
    segments: usize,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    if nonnegative_with(&radius, "radius", decisions)? == hyperreal::RealSign::Zero {
        return Ok(empty());
    }
    ellipse_points_with(radius.clone(), radius, segments, decisions)
}

fn translate_with(
    region: &CurveRegion2,
    x: Real,
    y: Real,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    Ok(decisions.exact_curve(|| {
        region.transform_affine(
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &x,
            &y,
        )
    })?)
}

fn contour_ring_sign(
    ring: &Contour2,
    decisions: &GeometryDecisions,
) -> Result<hyperlimit::Sign, ValidationError> {
    let points = ring
        .segments()
        .iter()
        .map(|segment| {
            hyperlimit::Point2::new(segment.start().x().clone(), segment.start().y().clone())
        })
        .collect::<Vec<_>>();
    decisions.decide(
        hyperlimit::ring_area_sign(&points, decisions.predicate_policy()),
        "extrusion side-ring signed area",
    )
}

fn exact_ratio(numerator: usize, denominator: usize) -> Result<Real, ValidationError> {
    let numerator = u64::try_from(numerator).map_err(|_| ValidationError::CapacityOverflow)?;
    let denominator =
        u64::try_from(denominator).map_err(|_| ValidationError::CapacityOverflow)?;
    Ok((Real::from(numerator) / Real::from(denominator))?)
}

fn exact_polar(radius: &Real, angle: Real) -> [Real; 2] {
    [
        radius.clone() * angle.clone().cos(),
        radius.clone() * angle.sin(),
    ]
}

fn repeat_exact_rotational_pattern(
    local_points: &[[Real; 2]],
    copies: usize,
) -> Result<Vec<[Real; 2]>, ValidationError> {
    let capacity = solid::capacity::<[Real; 2]>(copies.checked_mul(local_points.len()))?;
    let mut points = solid::reserved(capacity)?;
    for copy in 0..copies {
        let angle = Real::tau() * exact_ratio(copy, copies)?;
        let sine = angle.clone().sin();
        let cosine = angle.cos();
        points.extend(local_points.iter().map(|[x, y]| {
            [
                x.clone() * cosine.clone() - y.clone() * sine.clone(),
                x.clone() * sine.clone() + y.clone() * cosine.clone(),
            ]
        }));
    }
    Ok(points)
}

/// Empty filled curve topology.
pub fn empty() -> CurveRegion2 {
    CurveRegion2::empty()
}

/// Axis-aligned rectangle with its minimum corner at the origin.
///
/// Dimensions must be nonnegative. A certified zero dimension produces the
/// canonical empty set after both arguments are validated. Negative or
/// undecided dimensions fail; they never become successful empty geometry.
pub fn rectangle(width: Real, length: Real) -> Result<CurveRegion2, ValidationError> {
    rectangle_with_context(width, length, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Axis-aligned square with its minimum corner at the origin.
pub fn square(width: Real) -> Result<CurveRegion2, ValidationError> {
    rectangle(width.clone(), width)
}

/// Regular sampled circle.
///
/// The radius is nonnegative and at least three segments are required, even
/// for zero radius. Certification concerns the sampled polygon; no analytic
/// circle fidelity bound is implied.
pub fn circle(radius: Real, segments: usize) -> Result<CurveRegion2, ValidationError> {
    circle_with_context(radius, segments, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs a rectangle with the selected policy and retains predicate certainty.
pub fn rectangle_with_context(
    width: Real,
    height: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let region = rectangle_with(width, height, &decisions)?;
    Ok(decisions.finish(region))
}

/// Constructs a square with the selected policy and retains predicate certainty.
pub fn square_with_context(
    width: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    rectangle_with_context(width.clone(), width, context)
}

/// Constructs a sampled circle with the selected policy and retains predicate certainty.
pub fn circle_with_context(
    radius: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let region = circle_with(radius, segments, &decisions)?;
    Ok(decisions.finish(region))
}

/// Constructs a sampled ellipse with the selected policy and retains predicate certainty.
pub fn ellipse_with_context(
    width: Real,
    height: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    let width_sign = nonnegative_with(&width, "width", &decisions)?;
    let height_sign = nonnegative_with(&height, "height", &decisions)?;
    if width_sign == hyperreal::RealSign::Zero || height_sign == hyperreal::RealSign::Zero {
        return Ok(decisions.finish(empty()));
    }
    let two = Real::from(2_u8);
    let region = ellipse_points_with(
        (width / &two).expect("two is nonzero"),
        (height / two).expect("two is nonzero"),
        segments,
        &decisions,
    )?;
    Ok(decisions.finish(region))
}

/// Constructs a regular polygon with the selected policy and retains predicate certainty.
pub fn regular_ngon_with_context(
    sides: usize,
    radius: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    circle_with_context(radius, sides, context)
}

/// Right triangle with its right-angle corner at the origin.
pub fn right_triangle(width: Real, height: Real) -> Result<CurveRegion2, ValidationError> {
    right_triangle_with_context(width, height, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs a right triangle, retaining the certainty of dimension and topology decisions.
pub fn right_triangle_with_context(
    width: Real,
    height: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let width_sign = nonnegative_with(&width, "width", &decisions)?;
    let height_sign = nonnegative_with(&height, "height", &decisions)?;
    if width_sign == hyperreal::RealSign::Zero || height_sign == hyperreal::RealSign::Zero {
        return Ok(decisions.finish(empty()));
    }
    let region = ring_with(
        &[
            [Real::zero(), Real::zero()],
            [width, Real::zero()],
            [Real::zero(), height],
        ],
        &decisions,
    )?;
    Ok(decisions.finish(region))
}

/// Closed polygon from exact coordinate rows.
///
/// Requires a valid ring with at least three vertices. Use [`empty`] for
/// canonical absence; an empty or malformed ring is an error.
pub fn polygon(points: &[[Real; 2]]) -> Result<CurveRegion2, ValidationError> {
    polygon_with_context(points, &GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Publishes a filled ring with the selected policy and aggregate certainty.
pub fn polygon_with_context(
    points: &[[Real; 2]],
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let region = ring_with(points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Closed polygon from native Hypercurve points.
pub fn polygon_points(points: &[Point2]) -> Result<CurveRegion2, ValidationError> {
    let mut rows = solid::reserved(points.len())?;
    rows.extend(
        points
            .iter()
            .map(|point| [point.x().clone(), point.y().clone()]),
    );
    polygon(&rows)
}

/// Sampled ellipse.
pub fn ellipse(
    width: Real,
    height: Real,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    ellipse_with_context(width, height, segments, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Regular polygon.
pub fn regular_ngon(sides: usize, radius: Real) -> Result<CurveRegion2, ValidationError> {
    regular_ngon_with_context(sides, radius, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Filled arrow.
///
/// All four dimensions must be positive. Equal shaft and head widths share
/// their shoulder endpoints; only those proven duplicates are omitted.
pub fn arrow(
    shaft_length: Real,
    shaft_width: Real,
    head_length: Real,
    head_width: Real,
) -> Result<CurveRegion2, ValidationError> {
    arrow_with_context(
        shaft_length,
        shaft_width,
        head_length,
        head_width,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs the same sampled profile with explicit predicate policy and certainty.
pub fn arrow_with_context(
    shaft_length: Real,
    shaft_width: Real,
    head_length: Real,
    head_width: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    for (value, name) in [
        (&shaft_length, "shaft_length"),
        (&shaft_width, "shaft_width"),
        (&head_length, "head_length"),
        (&head_width, "head_width"),
    ] {
        positive_with(value, name, &decisions)?;
    }
    let shoulders = compare_with(&shaft_width, &head_width, &decisions)? != Ordering::Equal;
    let two = Real::from(2_u8);
    let half_shaft = (shaft_width / &two)?;
    let half_head = (head_width / &two)?;
    let tip = shaft_length.clone() + head_length;
    let mut points = solid::reserved(7)?;
    points.push([Real::zero(), -half_shaft.clone()]);
    points.push([shaft_length.clone(), -half_shaft.clone()]);
    if shoulders {
        points.push([shaft_length.clone(), -half_head.clone()]);
    }
    points.push([tip, Real::zero()]);
    if shoulders {
        points.push([shaft_length.clone(), half_head]);
    }
    points.push([shaft_length, half_shaft.clone()]);
    points.push([Real::zero(), half_shaft]);
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Filled trapezoid.
///
/// Widths and height must be positive. The horizontal top offset is unrestricted.
pub fn trapezoid(
    top_width: Real,
    bottom_width: Real,
    height: Real,
    top_offset: Real,
) -> Result<CurveRegion2, ValidationError> {
    trapezoid_with_context(
        top_width,
        bottom_width,
        height,
        top_offset,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs the same sampled profile with explicit predicate policy and certainty.
pub fn trapezoid_with_context(
    top_width: Real,
    bottom_width: Real,
    height: Real,
    top_offset: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    positive_with(&top_width, "top_width", &decisions)?;
    positive_with(&bottom_width, "bottom_width", &decisions)?;
    positive_with(&height, "height", &decisions)?;
    let region = ring_with(
        &[
            [Real::zero(), Real::zero()],
            [bottom_width, Real::zero()],
            [top_offset.clone() + top_width, height.clone()],
            [top_offset, height],
        ],
        &decisions,
    )?;
    Ok(decisions.finish(region))
}

/// Alternating-radius star polygon.
///
/// Requires at least three points and `outer_radius > inner_radius > 0`.
pub fn star(
    num_points: usize,
    outer_radius: Real,
    inner_radius: Real,
) -> Result<CurveRegion2, ValidationError> {
    star_with_context(
        num_points,
        outer_radius,
        inner_radius,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs the same sampled profile with explicit predicate policy and certainty.
pub fn star_with_context(
    num_points: usize,
    outer_radius: Real,
    inner_radius: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(num_points, 3, "num_points")?;
    let count = solid::capacity::<[Real; 2]>(num_points.checked_mul(2))?;
    positive_with(&inner_radius, "inner_radius", &decisions)?;
    if compare_with(&outer_radius, &inner_radius, &decisions)? != Ordering::Greater {
        return Err(ValidationError::InvalidArguments);
    }
    let denominator = Real::from(count as u64);
    let mut points = solid::reserved(count)?;
    for index in 0..count {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let radius = if index % 2 == 0 {
            &outer_radius
        } else {
            &inner_radius
        };
        points.push([
            radius.clone() * angle.clone().cos(),
            radius.clone() * angle.sin(),
        ]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Teardrop region.
///
/// Requires positive dimensions, `length > width / 2`, and at least two
/// semicircle chords. The sampled boundary and its tip retain exact coordinates.
pub fn teardrop(
    width: Real,
    length: Real,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    teardrop_with_context(width, length, segments, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the same sampled profile with explicit predicate policy and certainty.
pub fn teardrop_with_context(
    width: Real,
    length: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 2, "segments")?;
    let capacity = solid::capacity::<[Real; 2]>(segments.checked_add(2))?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&length, "length", &decisions)?;
    let radius = (width / Real::from(2_u8))?;
    if compare_with(&length, &radius, &decisions)? != Ordering::Greater {
        return Err(ValidationError::InvalidArguments);
    }
    let center_y = length - radius.clone();
    let mut points = solid::reserved(capacity)?;
    points.push([Real::zero(), Real::zero()]);
    for index in 0..=segments {
        let fraction = (Real::from(index as u64) / Real::from(segments as u64))?;
        let angle = Real::pi() * fraction;
        points.push([
            -radius.clone() * angle.clone().cos(),
            center_y.clone() + radius.clone() * angle.sin(),
        ]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Egg region.
///
/// Requires positive dimensions and at least three samples. Sample extrema are
/// normalized to the requested width and length; this does not certify an analytic surface.
pub fn egg(
    width: Real,
    length: Real,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    egg_with_context(width, length, segments, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the same sampled profile with explicit predicate policy and certainty.
pub fn egg_with_context(
    width: Real,
    length: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&length, "length", &decisions)?;
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let sine = angle.clone().sin();
        let cosine = angle.cos();
        let correction = (cosine.clone() * cosine.clone() / Real::from(5_u8))?;
        points.push([-sine, cosine + correction]);
    }
    let quarter_turns = segments / 4;
    let remainder = segments % 4;
    let min_x_index = quarter_turns + usize::from(remainder >= 2);
    let max_x_index = 3 * quarter_turns + ((3 * remainder + 2) / 4);
    let min_y_index = segments / 2;
    let min_x = points[min_x_index][0].clone();
    let max_x = points[max_x_index][0].clone();
    let min_y = points[min_y_index][1].clone();
    let max_y = points[0][1].clone();
    let x_scale = (width.clone() / (max_x.clone() - min_x.clone()))?;
    let y_scale = (length.clone() / (max_y.clone() - min_y.clone()))?;
    let x_midpoint = ((min_x + max_x) / Real::from(2_u8))?;
    let y_midpoint = ((min_y + max_y) / Real::from(2_u8))?;
    for [x, y] in &mut points {
        *x = (x.clone() - x_midpoint.clone()) * x_scale.clone();
        *y = (y.clone() - y_midpoint.clone()) * y_scale.clone();
    }
    let half_width = (width / Real::from(2_u8))?;
    let half_length = (length / Real::from(2_u8))?;
    points[min_x_index][0] = -half_width.clone();
    points[max_x_index][0] = half_width;
    if remainder == 2 {
        points[min_x_index - 1][0] = points[min_x_index][0].clone();
        points[max_x_index - 1][0] = points[max_x_index][0].clone();
    }
    points[min_y_index][1] = -half_length.clone();
    if segments % 2 == 1 {
        points[min_y_index + 1][1] = -half_length.clone();
    }
    points[0][1] = half_length;
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Rounded rectangle represented by exact samples and straight chords.
///
/// The radius is clamped to half the smaller dimension. For a positive radius
/// and `corner_segments = n > 0`, each quarter-circle has `n` uniform angular
/// intervals, and every nonzero straight side joins its tangent endpoints.
/// Only coincident endpoints are omitted. Zero radius or zero corner segments
/// selects the sharp rectangle. Dimensions and radius must be nonnegative.
pub fn rounded_rectangle(
    width: Real,
    height: Real,
    corner_radius: Real,
    corner_segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    rounded_rectangle_with_context(
        width,
        height,
        corner_radius,
        corner_segments,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs a rounded rectangle with the selected policy and aggregate certainty.
pub fn rounded_rectangle_with_context(
    width: Real,
    height: Real,
    corner_radius: Real,
    corner_segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let point_capacity = solid::capacity::<[Real; 2]>(
        corner_segments
            .checked_add(1)
            .and_then(|count| count.checked_mul(4)),
    )?;
    let width_sign = nonnegative_with(&width, "width", &decisions)?;
    let height_sign = nonnegative_with(&height, "height", &decisions)?;
    nonnegative_with(&corner_radius, "corner_radius", &decisions)?;
    if width_sign == hyperreal::RealSign::Zero || height_sign == hyperreal::RealSign::Zero {
        return Ok(decisions.finish(empty()));
    }
    let half_width = (&width / Real::from(2_u8)).expect("two is nonzero");
    let half_height = (&height / Real::from(2_u8)).expect("two is nonzero");
    let (radius, shared_x) = match compare_with(&corner_radius, &half_width, &decisions)? {
        Ordering::Greater => (half_width, true),
        Ordering::Equal => (corner_radius, true),
        Ordering::Less => (corner_radius, false),
    };
    let (radius, shared_x, shared_y) = match compare_with(&radius, &half_height, &decisions)? {
        // half_height < radius <= half_width also proves a nonzero horizontal side.
        Ordering::Greater => (half_height, false, true),
        Ordering::Equal => (radius, shared_x, true),
        Ordering::Less => (radius, shared_x, false),
    };
    if corner_segments == 0
        || decisions.sign(&radius, "corner radius")? == hyperreal::RealSign::Zero
    {
        return Ok(decisions.finish(rectangle_with(width, height, &decisions)?));
    }
    let half_pi = (Real::pi() / Real::from(2_u8)).expect("two is nonzero");
    let centers = [
        (radius.clone(), radius.clone(), Real::pi()),
        (
            width.clone() - radius.clone(),
            radius.clone(),
            Real::pi() + half_pi.clone(),
        ),
        (
            width.clone() - radius.clone(),
            height.clone() - radius.clone(),
            Real::zero(),
        ),
        (radius.clone(), height - radius.clone(), half_pi.clone()),
    ];
    let mut points = solid::reserved(point_capacity)?;
    let corner_count = centers.len();
    for (corner_index, (cx, cy, start)) in centers.into_iter().enumerate() {
        let shared_start = match corner_index {
            1 | 3 => shared_x,
            2 => shared_y,
            _ => false,
        };
        for index in 0..=corner_segments {
            // The clamp comparisons prove which tangent endpoints coincide.
            // Other adjacent corners are joined by a nonzero straight side,
            // whose two endpoints must both remain in the sampled boundary.
            if (shared_start && index == 0)
                || (shared_y && corner_index + 1 == corner_count && index == corner_segments)
            {
                continue;
            }
            let fraction = (Real::from(index as u64) / Real::from(corner_segments as u64))?;
            let angle = start.clone() + half_pi.clone() * fraction;
            points.push([
                cx.clone() + radius.clone() * angle.clone().cos(),
                cy.clone() + radius.clone() * angle.sin(),
            ]);
        }
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Squircle region.
///
/// Requires positive dimensions and at least three samples. The signed square
/// roots of sampled sine and cosine define the represented polygon.
pub fn squircle(
    width: Real,
    height: Real,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    squircle_with_context(width, height, segments, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the same sampled profile with explicit predicate policy and certainty.
pub fn squircle_with_context(
    width: Real,
    height: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&height, "height", &decisions)?;
    let rx = (width / Real::from(2_u8))?;
    let ry = (height / Real::from(2_u8))?;
    let signed_root = |value: Real| -> Result<Real, ValidationError> {
        let negative =
            decisions.sign(&value, "squircle sample sign")? == hyperreal::RealSign::Negative;
        let root = value.abs().sqrt()?;
        Ok(if negative { -root } else { root })
    };
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        points.push([
            rx.clone() * signed_root(angle.clone().cos())?,
            ry.clone() * signed_root(angle.sin())?,
        ]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Keyhole region.
pub fn keyhole(
    circle_radius: Real,
    handle_width: Real,
    handle_height: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&circle_radius, "circle_radius", &decisions)?;
    positive_with(&handle_width, "handle_width", &decisions)?;
    positive_with(&handle_height, "handle_height", &decisions)?;
    let handle_x = -(handle_width.clone() / Real::from(2_u8)).expect("two is nonzero");
    let handle = translate_with(
        &rectangle_with(handle_width, handle_height, &decisions)?,
        handle_x,
        Real::zero(),
        &decisions,
    )?;
    let operand = circle_with(circle_radius, segments, &decisions)?;
    let region =
        decisions.exact_curve(|| operand.boolean_region(&handle, BooleanOp::Union))?;
    Ok(decisions.finish(region))
}

/// Reuleaux polygon.
pub fn reuleaux(
    sides: usize,
    diameter: Real,
    circle_segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(sides, 3, "sides")?;
    solid::sample_minimum(circle_segments, 6, "circle_segments")?;
    solid::capacity::<[Real; 2]>(Some(circle_segments))?;
    if sides.is_multiple_of(2) {
        return Err(ValidationError::InvalidArguments);
    }
    let double_sides = sides
        .checked_mul(2)
        .ok_or(ValidationError::CapacityOverflow)?;
    positive_with(&diameter, "diameter", &decisions)?;
    let half_angle = (Real::pi() / Real::from(double_sides as u64))?;
    let circumradius = (diameter.clone() / (Real::from(2_u8) * half_angle.cos()))?;
    let denominator = Real::from(sides as u64);
    let mut result: Option<CurveRegion2> = None;
    for index in 0..sides {
        let angle = Real::tau()
            * (Real::from(index as u64) / &denominator)
                .expect("the validated side count is nonzero");
        let disk = translate_with(
            &circle_with(diameter.clone(), circle_segments, &decisions)?,
            circumradius.clone() * angle.clone().cos(),
            circumradius.clone() * angle.sin(),
            &decisions,
        )?;
        result = Some(match result {
            None => disk,
            Some(current) => decisions
                .exact_curve(|| current.boolean_region(&disk, BooleanOp::Intersection))?,
        });
    }
    Ok(decisions.finish(result.unwrap_or_else(empty)))
}

/// Annular region.
pub fn ring(
    inner_diameter: Real,
    thickness: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&inner_diameter, "inner_diameter", &decisions)?;
    positive_with(&thickness, "thickness", &decisions)?;
    let inner_radius = (inner_diameter / Real::from(2_u8))?;
    let inner = circle_with(inner_radius.clone(), segments, &decisions)?;
    let operand = circle_with(inner_radius + thickness, segments, &decisions)?;
    let region =
        decisions.exact_curve(|| operand.boolean_region(&inner, BooleanOp::Difference))?;
    Ok(decisions.finish(region))
}

/// Circular sector.
///
/// Requires positive radius, at least one chord, and a nonzero sweep of at most
/// 360 degrees in either direction. Native failures remain typed; a successfully
/// regularized zero-area sample represents the empty set.
/// A full turn preserves the existing full-circle sampling with at least three vertices.
pub fn pie_slice(
    radius: Real,
    start_angle_deg: Real,
    end_angle_deg: Real,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    pie_slice_with_context(
        radius,
        start_angle_deg,
        end_angle_deg,
        segments,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs the same sampled profile with explicit predicate policy and certainty.
pub fn pie_slice_with_context(
    radius: Real,
    start_angle_deg: Real,
    end_angle_deg: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 1, "segments")?;
    let capacity = solid::capacity::<[Real; 2]>(segments.checked_add(2))?;
    positive_with(&radius, "radius", &decisions)?;
    let sweep = end_angle_deg - start_angle_deg.clone();
    let absolute_sweep = sweep.clone().abs();
    positive_with(&absolute_sweep, "sweep", &decisions)?;
    match compare_with(&absolute_sweep, &Real::from(360_u16), &decisions)? {
        Ordering::Greater => return Err(ValidationError::InvalidArguments),
        Ordering::Equal => {
            let region =
                ellipse_points_with(radius.clone(), radius, segments.max(3), &decisions)?;
            return Ok(decisions.finish(region));
        },
        Ordering::Less => {},
    }
    let start = (start_angle_deg * Real::pi() / Real::from(180_u16))?;
    let sweep = (sweep * Real::pi() / Real::from(180_u16))?;
    let mut points = solid::reserved(capacity)?;
    points.push([Real::zero(), Real::zero()]);
    for index in 0..=segments {
        let fraction = (Real::from(index as u64) / Real::from(segments as u64))?;
        let angle = start.clone() + sweep.clone() * fraction;
        points.push([
            radius.clone() * angle.clone().cos(),
            radius.clone() * angle.sin(),
        ]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Heart region.
///
/// Requires positive dimensions and at least eight samples. Sample extrema are
/// normalized to the requested dimensions; topology and analytic fidelity are distinct.
pub fn heart(
    width: Real,
    height: Real,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    heart_with_context(width, height, segments, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the same sampled profile with explicit predicate policy and certainty.
pub fn heart_with_context(
    width: Real,
    height: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 8, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    let quarter_rounding_numerator = segments
        .checked_add(2)
        .ok_or(ValidationError::CapacityOverflow)?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&height, "height", &decisions)?;
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let sine = angle.clone().sin();
        let cosine = angle.cos();
        let cosine_squared = cosine.clone() * cosine.clone();
        points.push([
            Real::from(16_u8) * sine.clone() * sine.clone() * sine,
            Real::from(4_u8) + Real::from(19_u8) * cosine.clone()
                - Real::from(2_u8) * cosine_squared.clone()
                - Real::from(8_u8) * cosine_squared.clone() * cosine
                - Real::from(8_u8) * cosine_squared.clone() * cosine_squared,
        ]);
    }
    let max_x_index = quarter_rounding_numerator / 4;
    let min_x_index = (segments - max_x_index) % segments;
    let min_x = points[min_x_index][0].clone();
    let max_x = points[max_x_index][0].clone();
    let mut lower = 0;
    let mut upper = segments / 2;
    while lower < upper {
        let middle = lower + (upper - lower) / 2;
        match compare_with(&points[middle][1], &points[middle + 1][1], &decisions)? {
            Ordering::Less => lower = middle + 1,
            Ordering::Equal | Ordering::Greater => upper = middle,
        }
    }
    let min_y = points[segments / 2][1].clone();
    let max_y = points[lower][1].clone();
    let span_x = max_x - min_x.clone();
    let span_y = max_y - min_y.clone();
    for [x, y] in &mut points {
        *x = width.clone() * ((x.clone() - min_x.clone()) / span_x.clone())?;
        *y = height.clone() * ((y.clone() - min_y.clone()) / span_y.clone())?;
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Crescent region.
pub fn crescent(
    outer_radius: Real,
    inner_radius: Real,
    offset: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 6, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&inner_radius, "inner_radius", &decisions)?;
    if compare_with(&outer_radius, &inner_radius, &decisions)? != Ordering::Greater {
        return Err(ValidationError::InvalidArguments);
    }
    let outer = circle_with(outer_radius, segments, &decisions)?;
    let inner = translate_with(
        &circle_with(inner_radius, segments, &decisions)?,
        offset,
        Real::zero(),
        &decisions,
    )?;
    let region =
        decisions.exact_curve(|| outer.boolean_region(&inner, BooleanOp::Difference))?;
    Ok(decisions.finish(region))
}

/// Superformula-derived filled region.
///
/// Requires positive `a` and `b`, nonzero `n1`, at least three samples, and
/// defined scalar powers at every sample. Native scalar and topology failures remain typed.
#[allow(clippy::too_many_arguments)]
pub fn supershape(
    a: Real,
    b: Real,
    m: Real,
    n1: Real,
    n2: Real,
    n3: Real,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    supershape_with_context(a, b, m, n1, n2, n3, segments, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the same sampled profile with explicit predicate policy and certainty.
#[allow(clippy::too_many_arguments)]
pub fn supershape_with_context(
    a: Real,
    b: Real,
    m: Real,
    n1: Real,
    n2: Real,
    n3: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&a, "a", &decisions)?;
    positive_with(&b, "b", &decisions)?;
    let exponent = (Real::from(-1_i8) / n1)?;
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let theta = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let angle = (m.clone() * theta.clone() / Real::from(4_u8))?;
        let first = (angle.clone().cos().abs() / a.clone())?.pow(n2.clone())?;
        let second = (angle.sin().abs() / b.clone())?.pow(n3.clone())?;
        let radius = (first + second).pow(exponent.clone())?;
        points.push([radius.clone() * theta.clone().cos(), radius * theta.sin()]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Circular region with a rectangular keyway.
pub fn circle_with_keyway(
    radius: Real,
    segments: usize,
    key_width: Real,
    key_depth: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&radius, "radius", &decisions)?;
    positive_with(&key_width, "key_width", &decisions)?;
    positive_with(&key_depth, "key_depth", &decisions)?;
    let diameter = Real::from(2_u8) * radius.clone();
    if compare_with(&key_width, &diameter, &decisions)? != Ordering::Less
        || compare_with(&key_depth, &diameter, &decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }
    let key_y = -(key_width.clone() / Real::from(2_u8)).expect("two is nonzero");
    let cutter = translate_with(
        &rectangle_with(key_depth.clone(), key_width, &decisions)?,
        radius.clone() - key_depth,
        key_y,
        &decisions,
    )?;
    let operand = circle_with(radius, segments, &decisions)?;
    let region =
        decisions.exact_curve(|| operand.boolean_region(&cutter, BooleanOp::Difference))?;
    Ok(decisions.finish(region))
}

/// Circular region with one flat chord.
pub fn circle_with_flat(
    radius: Real,
    segments: usize,
    flat_distance: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&radius, "radius", &decisions)?;
    nonnegative_with(&flat_distance, "flat_distance", &decisions)?;
    if compare_with(&flat_distance, &radius, &decisions)? != Ordering::Less {
        return Err(ValidationError::InvalidArguments);
    }
    let diameter = Real::from(2_u8) * radius.clone();
    let clip = translate_with(
        &rectangle_with(diameter, radius.clone() + flat_distance.clone(), &decisions)?,
        -radius.clone(),
        -flat_distance,
        &decisions,
    )?;
    let operand = circle_with(radius, segments, &decisions)?;
    let region =
        decisions.exact_curve(|| operand.boolean_region(&clip, BooleanOp::Intersection))?;
    Ok(decisions.finish(region))
}

/// Circular region with two opposing flat chords.
pub fn circle_with_two_flats(
    radius: Real,
    segments: usize,
    flat_distance: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&radius, "radius", &decisions)?;
    nonnegative_with(&flat_distance, "flat_distance", &decisions)?;
    if compare_with(&flat_distance, &radius, &decisions)? != Ordering::Less {
        return Err(ValidationError::InvalidArguments);
    }
    let clip = translate_with(
        &rectangle_with(
            Real::from(2_u8) * radius.clone(),
            Real::from(2_u8) * flat_distance.clone(),
            &decisions,
        )?,
        -radius.clone(),
        -flat_distance,
        &decisions,
    )?;
    let operand = circle_with(radius, segments, &decisions)?;
    let region =
        decisions.exact_curve(|| operand.boolean_region(&clip, BooleanOp::Intersection))?;
    Ok(decisions.finish(region))
}

/// Sampled involute gear region with typed validation and construction failures.
///
/// Requires a positive module, at least four teeth, at least two flank segments,
/// pressure angle in `(0, 90)` degrees and nonnegative clearance/backlash.
/// Derived root radius and tooth widths must remain positive. The result is
/// the finite sampled polygon, without an analytic-fidelity error bound.
pub fn involute_gear(
    module: Real,
    teeth: usize,
    pressure_angle_degrees: Real,
    clearance: Real,
    backlash: Real,
    segments_per_flank: usize,
) -> Result<CurveRegion2, ValidationError> {
    involute_gear_with_context(
        module,
        teeth,
        pressure_angle_degrees,
        clearance,
        backlash,
        segments_per_flank,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// [`involute_gear`] with explicit predicate policy and local certainty.
pub fn involute_gear_with_context(
    module: Real,
    teeth: usize,
    pressure_angle_degrees: Real,
    clearance: Real,
    backlash: Real,
    segments_per_flank: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(teeth, 4, "teeth")?;
    solid::sample_minimum(segments_per_flank, 2, "segments_per_flank")?;
    positive_with(&module, "module", &decisions)?;
    nonnegative_with(&clearance, "clearance", &decisions)?;
    nonnegative_with(&backlash, "backlash", &decisions)?;
    positive_with(&pressure_angle_degrees, "pressure_angle_degrees", &decisions)?;
    if compare_with(&pressure_angle_degrees, &Real::from(90_u8), &decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }
    let region = sampled_involute_gear(
        &module,
        teeth,
        &pressure_angle_degrees,
        &clearance,
        &backlash,
        segments_per_flank,
        &decisions,
    )?;
    Ok(decisions.finish(region))
}

/// Sampled cycloidal gear region with typed validation and construction failures.
///
/// Requires positive module/generating radius, at least three teeth, at least
/// two flank segments and nonnegative clearance. Rejects first-lobe flanks
/// that meet or cross before the addendum circle or neighboring root gap.
/// The finite sampled polygon has no analytic-fidelity error bound.
pub fn cycloidal_gear(
    module: Real,
    teeth: usize,
    generating_radius: Real,
    clearance: Real,
    segments_per_flank: usize,
) -> Result<CurveRegion2, ValidationError> {
    cycloidal_gear_with_context(
        module,
        teeth,
        generating_radius,
        clearance,
        segments_per_flank,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// [`cycloidal_gear`] with explicit predicate policy and local certainty.
pub fn cycloidal_gear_with_context(
    module: Real,
    teeth: usize,
    generating_radius: Real,
    clearance: Real,
    segments_per_flank: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(teeth, 3, "teeth")?;
    solid::sample_minimum(segments_per_flank, 2, "segments_per_flank")?;
    positive_with(&generating_radius, "generating_radius", &decisions)?;
    positive_with(&module, "module", &decisions)?;
    nonnegative_with(&clearance, "clearance", &decisions)?;
    let region = sampled_cycloidal_gear(
        &module,
        teeth,
        &generating_radius,
        &clearance,
        segments_per_flank,
        &decisions,
    )?;
    Ok(decisions.finish(region))
}

/// Linear involute rack region, with at least one tooth and positive module.
///
/// Pressure angle must be in `(0, 90)` degrees, clearance and backlash
/// nonnegative, and the resulting tooth widths and root gaps positive.
pub fn involute_rack(
    module: Real,
    teeth: usize,
    pressure_angle_degrees: Real,
    clearance: Real,
    backlash: Real,
) -> Result<CurveRegion2, ValidationError> {
    involute_rack_with_context(
        module,
        teeth,
        pressure_angle_degrees,
        clearance,
        backlash,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// [`involute_rack`] with explicit predicate policy and local certainty.
pub fn involute_rack_with_context(
    module: Real,
    teeth: usize,
    pressure_angle_degrees: Real,
    clearance: Real,
    backlash: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(teeth, 1, "teeth")?;
    positive_with(&module, "module", &decisions)?;
    positive_with(&pressure_angle_degrees, "pressure_angle_degrees", &decisions)?;
    nonnegative_with(&clearance, "clearance", &decisions)?;
    nonnegative_with(&backlash, "backlash", &decisions)?;
    if compare_with(&pressure_angle_degrees, &Real::from(90_u8), &decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }
    let point_capacity = solid::capacity::<[Real; 2]>(
        teeth.checked_mul(6).and_then(|count| count.checked_add(2)),
    )?;
    let pressure_angle = (pressure_angle_degrees * Real::pi() / Real::from(180_u16))
        .expect("degrees denominator is nonzero");
    let pitch = Real::pi() * module.clone();
    let dedendum = module.clone()
        * (Real::from(5_u8) / Real::from(4_u8)).expect("four is nonzero")
        + clearance;
    let root = -dedendum.clone();
    let half_pitch = (pitch.clone() / Real::from(2_u8))?;
    let tooth_thickness = half_pitch - backlash;
    let half_thickness = (tooth_thickness.clone() / Real::from(2_u8)).expect("two is nonzero");
    let tangent = (pressure_angle.clone().sin() / pressure_angle.cos())?;
    let root_slant = dedendum * tangent.clone();
    let tip_slant = module.clone() * tangent;
    let tip_width = tooth_thickness.clone() - Real::from(2_u8) * tip_slant.clone();
    let root_space =
        pitch.clone() - tooth_thickness.clone() - Real::from(2_u8) * root_slant.clone();
    positive_with(&tooth_thickness, "tooth_thickness", &decisions)?;
    positive_with(&tip_width, "tip_width", &decisions)?;
    positive_with(&root_space, "root_space", &decisions)?;
    let first_x = -half_thickness.clone() - root_slant.clone();
    let mut points = solid::reserved(point_capacity)?;
    points.push([first_x, root.clone()]);
    for tooth in 0..teeth {
        let center = Real::from(tooth as u64) * pitch.clone();
        let left_pitch = center.clone() - half_thickness.clone();
        let right_pitch = center + half_thickness.clone();
        points.extend([
            [left_pitch.clone(), Real::zero()],
            [left_pitch + tip_slant.clone(), module.clone()],
            [right_pitch.clone() - tip_slant.clone(), module.clone()],
            [right_pitch.clone(), Real::zero()],
            [right_pitch + root_slant.clone(), root.clone()],
        ]);
        if tooth + 1 < teeth {
            let next_left =
                Real::from((tooth + 1) as u64) * pitch.clone() - half_thickness.clone();
            points.push([next_left - root_slant.clone(), root.clone()]);
        }
    }
    let left = points.first().expect("validated rack has a left root")[0].clone();
    let right = points.last().expect("validated rack has a right root")[0].clone();
    let backing = root - module;
    points.push([right, backing.clone()]);
    points.push([left, backing]);
    Ok(decisions.finish(ring_with(&points, &decisions)?))
}

/// Linear cycloidal rack region.
///
/// Each tooth joins an addendum cycloid above the pitch line to a dedendum
/// cycloid below it on both sides. Each rolling radius is twice its
/// corresponding standard height, so every sampled flank spans one sixth turn
/// and meets a horizontal tip or root land.
///
/// Requires a positive module, at least one tooth, at least four flank
/// segments and nonnegative clearance. A closed root land is a typed error.
/// The finite sampled polygon has no analytic-fidelity error bound.
pub fn cycloidal_rack(
    module: Real,
    teeth: usize,
    clearance: Real,
    segments_per_flank: usize,
) -> Result<CurveRegion2, ValidationError> {
    cycloidal_rack_with_context(
        module,
        teeth,
        clearance,
        segments_per_flank,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// [`cycloidal_rack`] with explicit predicate policy and local certainty.
pub fn cycloidal_rack_with_context(
    module: Real,
    teeth: usize,
    clearance: Real,
    segments_per_flank: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(teeth, 1, "teeth")?;
    solid::sample_minimum(segments_per_flank, 4, "segments_per_flank")?;
    positive_with(&module, "module", &decisions)?;
    nonnegative_with(&clearance, "clearance", &decisions)?;
    let sample_capacity = solid::capacity::<[Real; 2]>(segments_per_flank.checked_add(1))?;
    let tooth_capacity = solid::capacity::<[Real; 2]>(
        segments_per_flank
            .checked_mul(4)
            .and_then(|count| count.checked_add(4)),
    )?;
    let top_capacity = solid::capacity::<[Real; 2]>(
        teeth
            .checked_mul(tooth_capacity - 1)
            .and_then(|count| count.checked_add(1)),
    )?;
    let point_capacity = solid::capacity::<[Real; 2]>(top_capacity.checked_add(2))?;
    let two = Real::from(2_u8);
    let three = Real::from(3_u8);
    let four = Real::from(4_u8);
    let pitch = Real::pi() * module.clone();
    let half_pitch = (pitch.clone() / two.clone()).expect("two is nonzero");
    let quarter_pitch = (pitch.clone() / four.clone()).expect("four is nonzero");
    let dedendum =
        module.clone() * (Real::from(5_u8) / four).expect("four is nonzero") + clearance;
    let root = -dedendum.clone();
    let flank_sweep = (Real::pi() / three).expect("three is nonzero");
    let run_factor = two.clone() * (flank_sweep.clone() - flank_sweep.clone().sin());
    let face_run = module.clone() * run_factor.clone();
    let root_run = dedendum.clone() * run_factor;
    if compare_with(&face_run, &quarter_pitch, &decisions)? != Ordering::Less
        || compare_with(&root_run, &quarter_pitch, &decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }

    let mut unit_flank = solid::reserved(sample_capacity)?;
    for sample in 0..=segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        let theta = flank_sweep.clone() * fraction;
        unit_flank.push([
            two.clone() * (theta.clone() - theta.clone().sin()),
            two.clone() * (Real::one() - theta.cos()),
        ]);
    }

    let mut tooth_points = solid::reserved(tooth_capacity)?;
    tooth_points.push([-half_pitch.clone(), root.clone()]);
    tooth_points.extend(unit_flank.iter().rev().map(|[run, rise]| {
        [
            -quarter_pitch.clone() - dedendum.clone() * run.clone(),
            -(dedendum.clone() * rise.clone()),
        ]
    }));
    tooth_points.extend(unit_flank.iter().skip(1).map(|[run, rise]| {
        [
            -quarter_pitch.clone() + module.clone() * run.clone(),
            module.clone() * rise.clone(),
        ]
    }));
    tooth_points.push([quarter_pitch.clone() - face_run, module.clone()]);
    tooth_points.extend(
        unit_flank[..segments_per_flank]
            .iter()
            .rev()
            .map(|[run, rise]| {
                [
                    quarter_pitch.clone() - module.clone() * run.clone(),
                    module.clone() * rise.clone(),
                ]
            }),
    );
    tooth_points.extend(unit_flank.iter().skip(1).map(|[run, rise]| {
        [
            quarter_pitch.clone() + dedendum.clone() * run.clone(),
            -(dedendum.clone() * rise.clone()),
        ]
    }));
    tooth_points.push([half_pitch.clone(), root.clone()]);

    let mut top = solid::reserved(top_capacity)?;
    for tooth in 0..teeth {
        let offset = Real::from(tooth as u64) * pitch.clone();
        for (point_index, point) in tooth_points.iter().enumerate() {
            if tooth > 0 && point_index == 0 {
                continue;
            }
            top.push([offset.clone() + point[0].clone(), point[1].clone()]);
        }
    }

    let mut points = solid::reserved(point_capacity)?;
    let backing = root.clone() - module;
    let left = top.first().expect("validated rack has a left root")[0].clone();
    let right = top.last().expect("validated rack has a right root")[0].clone();
    points.push([left, backing.clone()]);
    points.push([right, backing]);
    points.extend(top.into_iter().rev());
    Ok(decisions.finish(ring_with(&points, &decisions)?))
}

/// Sampled NACA four-digit airfoil region with typed failures.
///
/// Camber and camber-position digits are in `[0, 10)`, thickness in `(0, 100)`,
/// chord positive, and `samples >= 10`. Positive camber requires a positive
/// camber position. This preserves the existing sampled ring, including its
/// represented binary64 coefficient values and trailing-edge closure; it does
/// not certify an analytic-fidelity error bound.
pub fn airfoil_naca4(
    max_camber: Real,
    camber_position: Real,
    thickness: Real,
    chord: Real,
    samples: usize,
) -> Result<CurveRegion2, ValidationError> {
    airfoil_naca4_with_context(
        max_camber,
        camber_position,
        thickness,
        chord,
        samples,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// [`airfoil_naca4`] with explicit predicate policy and local certainty.
pub fn airfoil_naca4_with_context(
    max_camber: Real,
    camber_position: Real,
    thickness: Real,
    chord: Real,
    samples: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(samples, 10, "samples")?;
    let cambered = nonnegative_with(&max_camber, "max_camber", &decisions)?
        == hyperreal::RealSign::Positive;
    nonnegative_with(&camber_position, "camber_position", &decisions)?;
    positive_with(&thickness, "thickness", &decisions)?;
    positive_with(&chord, "chord", &decisions)?;
    if compare_with(&max_camber, &Real::from(10_u8), &decisions)? != Ordering::Less
        || compare_with(&camber_position, &Real::from(10_u8), &decisions)? != Ordering::Less
        || compare_with(&thickness, &Real::from(100_u8), &decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }
    let m = (max_camber / Real::from(100_u8))?;
    let p = (camber_position / Real::from(10_u8))?;
    let t = (thickness / Real::from(100_u8))?;
    if cambered {
        positive_with(&p, "camber_position", &decisions)?;
    }
    let coefficient = |value: f64| Real::try_from(value).expect("finite NACA coefficient");
    let sample_capacity = solid::capacity::<[Real; 2]>(samples.checked_add(1))?;
    let point_capacity = solid::capacity::<[Real; 2]>(samples.checked_mul(2))?;
    let mut upper = solid::reserved(point_capacity)?;
    let mut lower = solid::reserved(sample_capacity)?;
    for index in 0..=samples {
        let x = exact_ratio(index, samples)?;
        let x2 = x.clone() * x.clone();
        let x3 = x2.clone() * x.clone();
        let x4 = x3.clone() * x.clone();
        let root_x = x.clone().sqrt()?;
        let yt = Real::from(5_u8)
            * t.clone()
            * (coefficient(0.2969) * root_x
                - coefficient(0.1260) * x.clone()
                - coefficient(0.3516) * x2.clone()
                + coefficient(0.2843) * x3
                - coefficient(0.1015) * x4);
        let (yc, dy) = if cambered {
            let x_ordering = compare_with(&x, &p, &decisions)?;
            if x_ordering == Ordering::Less {
                let p2 = p.clone() * p.clone();
                let y_factor = (m.clone() / p2.clone())?;
                let slope_factor = (Real::from(2_u8) * m.clone() / p2)?;
                (
                    y_factor * (Real::from(2_u8) * p.clone() * x.clone() - x2.clone()),
                    slope_factor * (p.clone() - x.clone()),
                )
            } else {
                let q = Real::one() - p.clone();
                let denominator = q.clone() * q;
                let y_factor = (m.clone() / denominator.clone())?;
                let slope_factor = (Real::from(2_u8) * m.clone() / denominator)?;
                (
                    y_factor
                        * (Real::one() - Real::from(2_u8) * p.clone()
                            + Real::from(2_u8) * p.clone() * x.clone()
                            - x2.clone()),
                    slope_factor * (p.clone() - x.clone()),
                )
            }
        } else {
            (Real::zero(), Real::zero())
        };
        let theta = dy.atan()?;
        let sin_theta = theta.clone().sin();
        let cos_theta = theta.cos();
        upper.push([
            chord.clone() * (x.clone() - yt.clone() * sin_theta.clone()),
            chord.clone() * (yc.clone() + yt.clone() * cos_theta.clone()),
        ]);
        lower.push([
            chord.clone() * (x + yt.clone() * sin_theta),
            chord.clone() * (yc - yt * cos_theta),
        ]);
    }
    upper.extend(lower.into_iter().rev().skip(1).take(samples - 1));
    Ok(decisions.finish(ring_with(&upper, &decisions)?))
}

fn sampled_involute_gear(
    module: &Real,
    teeth: usize,
    pressure_angle_degrees: &Real,
    clearance: &Real,
    backlash: &Real,
    segments_per_flank: usize,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    let tooth_capacity = solid::capacity::<[Real; 2]>(
        segments_per_flank
            .checked_mul(4)
            .and_then(|count| count.checked_add(3)),
    )?;
    solid::capacity::<[Real; 2]>(teeth.checked_mul(tooth_capacity))?;
    let two = Real::from(2_u8);
    let four = Real::from(4_u8);
    let tooth_count = Real::from(teeth as u64);
    let pressure_angle = (pressure_angle_degrees.clone() * Real::pi() / Real::from(180_u16))?;
    let pitch_radius = (module.clone() * tooth_count.clone() / two.clone())?;
    let base_radius = pitch_radius.clone() * pressure_angle.cos();
    let outer_radius = pitch_radius.clone() + module.clone();
    let root_radius = pitch_radius.clone()
        - (module.clone() * Real::from(5_u8) / four.clone())?
        - clearance.clone();
    let angular_pitch = (Real::tau() / tooth_count)?;
    let backlash_angle = (backlash.clone() / pitch_radius.clone())?;
    let half_tooth_angle = (((angular_pitch.clone() / two.clone())? - backlash_angle) / two)?;
    positive_with(&base_radius, "base_radius", decisions)?;
    positive_with(&root_radius, "root_radius", decisions)?;
    positive_with(&half_tooth_angle, "half_tooth_angle", decisions)?;
    let involute_angle = |radius: &Real| -> Result<Real, ValidationError> {
        let ratio = (radius.clone() / base_radius.clone())?;
        let parameter = (ratio.clone() * ratio - Real::one()).sqrt()?;
        Ok(parameter.clone() - parameter.atan()?)
    };
    let (flank_start_radius, has_root_transition) =
        match compare_with(&root_radius, &base_radius, decisions)? {
            Ordering::Less => (base_radius.clone(), true),
            Ordering::Equal | Ordering::Greater => (root_radius.clone(), false),
        };
    let pitch_involute = involute_angle(&pitch_radius)?;
    let start_involute = involute_angle(&flank_start_radius)?;
    let outer_involute = involute_angle(&outer_radius)?;
    let offset = half_tooth_angle + pitch_involute;
    let right_start = start_involute.clone() - offset.clone();
    let left_start = offset.clone() - start_involute.clone();
    let right_tip = outer_involute.clone() - offset.clone();
    let left_tip = offset.clone() - outer_involute.clone();
    if compare_with(&right_start, &left_start, decisions)? != Ordering::Less
        || compare_with(&right_tip, &left_tip, decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }

    let mut flank_samples = solid::reserved(segments_per_flank)?;
    for sample in 1..segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        let radius = flank_start_radius.clone()
            + fraction * (outer_radius.clone() - flank_start_radius.clone());
        flank_samples.push((radius.clone(), involute_angle(&radius)?));
    }
    flank_samples.push((outer_radius.clone(), outer_involute));

    let mut tooth = solid::reserved(tooth_capacity)?;
    tooth.push(exact_polar(&root_radius, right_start.clone()));
    if has_root_transition {
        tooth.push(exact_polar(&flank_start_radius, right_start.clone()));
    }
    for (radius, angle) in &flank_samples {
        tooth.push(exact_polar(radius, angle.clone() - offset.clone()));
    }
    for sample in 1..=segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        tooth.push(exact_polar(
            &outer_radius,
            right_tip.clone() + fraction * (left_tip.clone() - right_tip.clone()),
        ));
    }
    for (radius, angle) in flank_samples[..flank_samples.len() - 1].iter().rev() {
        tooth.push(exact_polar(radius, offset.clone() - angle.clone()));
    }
    tooth.push(exact_polar(
        &flank_start_radius,
        offset.clone() - start_involute,
    ));
    if has_root_transition {
        tooth.push(exact_polar(&root_radius, left_start.clone()));
    }
    let next_right = angular_pitch + right_start;
    for sample in 1..segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        tooth.push(exact_polar(
            &root_radius,
            left_start.clone() + fraction * (next_right.clone() - left_start.clone()),
        ));
    }
    let points = repeat_exact_rotational_pattern(&tooth, teeth)?;
    ring_with(&points, decisions)
}

fn sampled_cycloidal_gear(
    module: &Real,
    teeth: usize,
    generating_radius: &Real,
    clearance: &Real,
    segments_per_flank: usize,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    let tooth_capacity = solid::capacity::<[Real; 2]>(
        segments_per_flank
            .checked_mul(6)
            .and_then(|count| count.checked_add(2)),
    )?;
    solid::capacity::<[Real; 2]>(teeth.checked_mul(tooth_capacity))?;
    let two = Real::from(2_u8);
    let pitch_radius = (module.clone() * Real::from(teeth as u64) / two.clone())?;
    let outer_radius = pitch_radius.clone() + module.clone();
    let root_radius = pitch_radius.clone()
        - (module.clone() * Real::from(5_u8) / Real::from(4_u8))?
        - clearance.clone();
    let twice_generator = two.clone() * generating_radius.clone();
    positive_with(&root_radius, "root_radius", decisions)?;
    if compare_with(&twice_generator, &pitch_radius, decisions)? != Ordering::Less {
        return Err(ValidationError::InvalidArguments);
    }
    let epicycle_radius = pitch_radius.clone() + generating_radius.clone();
    let hypocycle_radius = pitch_radius.clone() - generating_radius.clone();
    if compare_with(
        &outer_radius,
        &(pitch_radius.clone() + twice_generator.clone()),
        decisions,
    )? == Ordering::Greater
    {
        return Err(ValidationError::InvalidArguments);
    }
    let generated_root_radius = pitch_radius.clone() - twice_generator;
    let flank_root_radius =
        match compare_with(&root_radius, &generated_root_radius, decisions)? {
            Ordering::Less => generated_root_radius,
            Ordering::Equal | Ordering::Greater => root_radius.clone(),
        };
    let square = |value: &Real| value.clone() * value.clone();
    let tip_phase = ((square(&epicycle_radius) + square(generating_radius)
        - square(&outer_radius))
        / (two.clone() * generating_radius.clone() * epicycle_radius.clone()))?
    .acos()?;
    let tip_parameter =
        (tip_phase.clone() * generating_radius.clone() / pitch_radius.clone())?;
    let root_phase = ((square(&flank_root_radius)
        - square(&hypocycle_radius)
        - square(generating_radius))
        / (two.clone() * generating_radius.clone() * hypocycle_radius.clone()))?
    .acos()?;
    let root_parameter =
        (root_phase.clone() * generating_radius.clone() / pitch_radius.clone())?;
    let epicycle_ratio = (epicycle_radius.clone() / generating_radius.clone())?;
    let hypocycle_ratio = (hypocycle_radius.clone() / generating_radius.clone())?;
    let angular_pitch = (Real::tau() / Real::from(teeth as u64))?;
    let pitch_half_thickness = (angular_pitch.clone() / Real::from(4_u8))?;
    let epicycloid = |parameter: Real| {
        let carrier = parameter.clone();
        let rolling = epicycle_ratio.clone() * parameter;
        [
            epicycle_radius.clone() * carrier.clone().cos()
                - generating_radius.clone() * rolling.clone().cos(),
            epicycle_radius.clone() * carrier.sin()
                - generating_radius.clone() * rolling.sin(),
        ]
    };
    let hypocycloid = |parameter: Real| {
        let carrier = parameter.clone();
        let rolling = hypocycle_ratio.clone() * parameter;
        [
            hypocycle_radius.clone() * carrier.clone().cos()
                + generating_radius.clone() * rolling.clone().cos(),
            hypocycle_radius.clone() * carrier.sin()
                - generating_radius.clone() * rolling.sin(),
        ]
    };
    let sample_capacity = solid::capacity::<[Real; 2]>(segments_per_flank.checked_add(1))?;
    let mut epi = solid::reserved(sample_capacity)?;
    let mut hypo = solid::reserved(sample_capacity)?;
    for sample in 0..=segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        epi.push(epicycloid(tip_parameter.clone() * fraction.clone()));
        hypo.push(hypocycloid(-(root_parameter.clone() * fraction)));
    }

    let limit_sine = pitch_half_thickness.clone().sin();
    let limit_cosine = pitch_half_thickness.clone().cos();
    let precedes_tooth_bisector = |[x, y]: &[Real; 2]| -> Result<bool, ValidationError> {
        Ok(compare_with(
            &(y.clone() * limit_cosine.clone() - x.clone() * limit_sine.clone()),
            &Real::zero(),
            decisions,
        )? == Ordering::Less)
    };
    let negative_root_endpoint = hypo.last().expect("validated flank has an endpoint");
    let positive_root_endpoint = [
        negative_root_endpoint[0].clone(),
        -negative_root_endpoint[1].clone(),
    ];
    if !precedes_tooth_bisector(epi.last().expect("validated flank has an endpoint"))?
        || !precedes_tooth_bisector(&positive_root_endpoint)?
    {
        return Err(ValidationError::InvalidArguments);
    }

    let flank_rotation = -pitch_half_thickness;
    let flank_sine = flank_rotation.clone().sin();
    let flank_cosine = flank_rotation.cos();
    let rotate = |[x, y]: &[Real; 2]| {
        [
            x.clone() * flank_cosine.clone() - y.clone() * flank_sine.clone(),
            x.clone() * flank_sine.clone() + y.clone() * flank_cosine.clone(),
        ]
    };
    let mut right_epi = solid::reserved(sample_capacity)?;
    right_epi.extend(epi.iter().map(rotate));
    let mut right_hypo = solid::reserved(sample_capacity)?;
    right_hypo.extend(hypo.iter().map(rotate));
    let right_tip_point = right_epi.last().expect("validated flank has an endpoint");
    let right_root_point = right_hypo.last().expect("validated flank has an endpoint");
    let right_tip = right_tip_point[1].clone().atan2(right_tip_point[0].clone());
    let right_root = right_root_point[1].clone().atan2(right_root_point[0].clone());
    let left_tip = -right_tip.clone();
    let left_root = -right_root.clone();
    let next_right_root = angular_pitch.clone() + right_root.clone();
    let mut tooth = solid::reserved(tooth_capacity)?;
    let has_root_transition =
        compare_with(&flank_root_radius, &root_radius, decisions)? == Ordering::Greater;
    if has_root_transition {
        tooth.push(exact_polar(&root_radius, right_root.clone()));
    }
    tooth.extend(right_hypo.iter().rev().cloned());
    tooth.extend(right_epi.iter().skip(1).cloned());
    for sample in 1..=segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        tooth.push(exact_polar(
            &outer_radius,
            right_tip.clone() + fraction * (left_tip.clone() - right_tip.clone()),
        ));
    }
    tooth.extend(
        right_epi[..segments_per_flank]
            .iter()
            .rev()
            .map(|[x, y]| [x.clone(), -y.clone()]),
    );
    tooth.extend(
        right_hypo
            .iter()
            .skip(1)
            .map(|[x, y]| [x.clone(), -y.clone()]),
    );
    if has_root_transition {
        tooth.push(exact_polar(&root_radius, left_root.clone()));
    }
    for sample in 1..segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        tooth.push(exact_polar(
            &root_radius,
            left_root.clone() + fraction * (next_right_root.clone() - left_root.clone()),
        ));
    }
    let points = repeat_exact_rotational_pattern(&tooth, teeth)?;
    ring_with(&points, decisions)
}

/// Exact arbitrary-degree Bezier path.
///
/// Closed controls should be passed to [`bezier_region`] instead.
/// Invalid arguments and native construction failures retain typed diagnostics.
pub fn bezier_path(
    control: &[[Real; 2]],
    display_segments: usize,
) -> Result<CurvePath2, ValidationError> {
    bezier_path_with_context(control, display_segments, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs a native Bezier path with the selected policy and local certainty.
///
/// The positive display count does not sample or replace the exact control net.
pub fn bezier_path_with_context(
    control: &[[Real; 2]],
    display_segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurvePath2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if control.len() < 2 || display_segments < 1 {
        return Err(ValidationError::InvalidArguments);
    }
    let points = control
        .iter()
        .map(|point| Point2::new(point[0].clone(), point[1].clone()))
        .collect::<Vec<_>>();
    let path = decisions.exact_curve(|| -> Result<_, ValidationError> {
        let curve = match points.as_slice() {
            [start, end] => Curve2::from(LineSeg2::try_new(start.clone(), end.clone())?),
            [start, control, end] => Curve2::from(QuadraticBezier2::new(
                start.clone(),
                control.clone(),
                end.clone(),
            )),
            [start, control1, control2, end] => Curve2::from(CubicBezier2::new(
                start.clone(),
                control1.clone(),
                control2.clone(),
                end.clone(),
            )),
            _ => Curve2::from(RationalBezier2::try_new(
                points,
                vec![Real::one(); control.len()],
            )?),
        };
        Ok(CurvePath2::try_new(vec![curve])?)
    })?;
    Ok(decisions.finish(path))
}

/// Closed arbitrary-degree Bezier boundary as a filled region.
///
/// Construction failures are errors; a successfully regularized retraced
/// boundary can still represent the empty set.
pub fn bezier_region(
    control: &[[Real; 2]],
    display_segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    bezier_region_with_context(control, display_segments, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Publishes a closed Bezier boundary with the selected policy and aggregate certainty.
pub fn bezier_region_with_context(
    control: &[[Real; 2]],
    display_segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let path = decisions.consume_geometry(bezier_path_with_context(
        control,
        display_segments,
        context,
    )?);
    if !decisions.exact_curve(|| path.start().coincides_with(&path.end()))? {
        return Err(ValidationError::InvalidArguments);
    }
    let region = decisions.exact_curve(|| {
        CurveRegion2::try_from_boundary_paths(
            std::slice::from_ref(&path),
            hypercurve::FillRule::EvenOdd,
        )
    })?;
    Ok(decisions.finish(region))
}

/// Exact open-uniform polynomial B-spline path.
pub fn bspline_path(
    control: &[[Real; 2]],
    degree: usize,
    display_segments_per_span: usize,
) -> Result<CurvePath2, ValidationError> {
    bspline_path_with_context(
        control,
        degree,
        display_segments_per_span,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs a B-spline with the selected policy and aggregate certainty.
pub fn bspline_path_with_context(
    control: &[[Real; 2]],
    degree: usize,
    display_segments_per_span: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurvePath2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let minimum_control_count = degree
        .checked_add(1)
        .ok_or(ValidationError::InvalidArguments)?;
    if control.len() < minimum_control_count || display_segments_per_span < 1 {
        return Err(ValidationError::InvalidArguments);
    }
    let n = control.len() - 1;
    let m = n
        .checked_add(degree)
        .and_then(|count| count.checked_add(1))
        .ok_or(ValidationError::InvalidArguments)?;
    let span_count = n - degree + 1;
    let knots = (0..=m)
        .map(|index| {
            if index <= degree {
                Real::zero()
            } else if index >= m - degree {
                Real::from(span_count as u64)
            } else {
                Real::from((index - degree) as u64)
            }
        })
        .collect();
    let points = control
        .iter()
        .map(|point| Point2::new(point[0].clone(), point[1].clone()))
        .collect();
    let spline =
        decisions.exact_curve(|| PolynomialSplineCurve2::try_new(degree, points, knots))?;
    let path = decisions.exact_curve(|| CurvePath2::try_new(vec![Curve2::from(spline)]))?;
    Ok(decisions.finish(path))
}

/// Returns the exact native line-string view of [`hilbert_paths`].
///
/// This geometry-only convenience omits trim contact/parameter records. Use
/// [`hilbert_paths`] when those records or selected algebraic endpoints are
/// needed. A fragment without represented `Real` endpoints returns a typed
/// unsupported error; it is never replaced by a display chord or omitted.
pub fn hilbert_strings(
    boundary: &CurveRegion2,
    order: usize,
    padding: Real,
) -> Result<Vec<CurveString2>, ValidationError> {
    hilbert_strings_with_context(boundary, order, padding, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// [`hilbert_strings`] with the selected policy and aggregate local certainty.
pub fn hilbert_strings_with_context(
    boundary: &CurveRegion2,
    order: usize,
    padding: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<Vec<CurveString2>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let paths = decisions
        .consume_geometry(hilbert_paths_with_context(boundary, order, padding, context)?);
    let mut strings = solid::reserved(paths.len())?;
    for path in paths {
        let mut segments = solid::reserved(path.fragments().len())?;
        for fragment in path.fragments() {
            let curve = fragment.trim_fragment().curve();
            let line = if let Some(CurveGeometry2::Line(line)) = curve.geometry() {
                line.clone()
            } else {
                // Every source curve constructed by hilbert_paths is a line.
                // A native range wrapper may retain represented endpoints;
                // copy those exact values without reconstructing selected roots.
                let start = curve.start();
                let end = curve.end();
                let (Some(start), Some(end)) = (start.coordinates(), end.coordinates()) else {
                    return Err(ValidationError::UnsupportedPrimitive {
                        reason: "Hilbert trim requires retained exact endpoints; use hilbert_paths",
                    });
                };
                decisions.exact_curve(|| LineSeg2::try_new(start.clone(), end.clone()))?
            };
            segments.push(Segment2::Line(line));
        }
        strings.push(decisions.exact_curve(|| CurveString2::try_new(segments))?);
    }
    Ok(decisions.finish(strings))
}

/// Clips a finite Hilbert cell-center polyline to a region's closed filled set.
///
/// Native trim records retain maximal connected runs in traversal order,
/// source segment indices and parameters, and boundary contact evidence.
/// Positive-length boundary overlaps are included; isolated contacts add no
/// piece. Holes and disconnected components use the native exact trim kernel.
/// Undecided or unsupported operations return typed errors without partial output.
///
/// `order` must be in `1..=10`, and `padding` must be nonnegative. Padding
/// insets the grid's bounding rectangle, not every boundary of the region.
/// Valid empty regions return an empty collection. Nonempty padded bounds must
/// have positive width and height. The order cap limits generated points to
/// 1,048,576; it is not a total-work, coefficient-growth, heap-byte or
/// cancellation budget for native trimming.
pub fn hilbert_paths(
    boundary: &CurveRegion2,
    order: usize,
    padding: Real,
) -> Result<Vec<CurvePathRegionTrim2>, ValidationError> {
    hilbert_paths_with_context(boundary, order, padding, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// [`hilbert_paths`] with policy-explicit trimming and local certainty.
///
/// The receipt covers consumed bounds, construction and trim decisions; it
/// does not certify the intent of an earlier approximate region construction.
pub fn hilbert_paths_with_context(
    boundary: &CurveRegion2,
    order: usize,
    padding: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<Vec<CurvePathRegionTrim2>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if order == 0
        || order > 10
        || !matches!(
            decisions.sign(&padding, "Hilbert padding")?,
            hyperreal::RealSign::Positive | hyperreal::RealSign::Zero
        )
    {
        return Err(ValidationError::InvalidArguments);
    }
    let Some(bounds) =
        decisions.consume_geometry(try_bounding_box_with_context(boundary, context)?)
    else {
        return Ok(decisions.finish(Vec::new()));
    };
    let width =
        bounds.maxs.x.clone() - bounds.mins.x.clone() - Real::from(2_u8) * padding.clone();
    let height =
        bounds.maxs.y.clone() - bounds.mins.y.clone() - Real::from(2_u8) * padding.clone();
    if !matches!(
        decisions.sign(&width, "Hilbert width")?,
        hyperreal::RealSign::Positive
    ) || !matches!(
        decisions.sign(&height, "Hilbert height")?,
        hyperreal::RealSign::Positive
    ) {
        return Err(ValidationError::InvalidArguments);
    }
    const fn rotate_quadrant(size: u64, x: &mut u64, y: &mut u64, rx: u64, ry: u64) {
        if ry == 0 {
            if rx == 1 {
                *x = size - 1 - *x;
                *y = size - 1 - *y;
            }
            std::mem::swap(x, y);
        }
    }
    const fn point(size: u64, mut distance: u64) -> (u64, u64) {
        let (mut x, mut y) = (0, 0);
        let mut scale = 1;
        while scale < size {
            let rx = (distance / 2) & 1;
            let ry = (distance ^ rx) & 1;
            rotate_quadrant(scale, &mut x, &mut y, rx, ry);
            x += scale * rx;
            y += scale * ry;
            distance /= 4;
            scale *= 2;
        }
        (x, y)
    }
    let size = 1_u64 << order;
    let denominator = Real::from(size);
    let half = (Real::one() / Real::from(2_u8)).expect("two is nonzero");
    let origin_x = bounds.mins.x.clone() + padding.clone();
    let origin_y = bounds.mins.y.clone() + padding;
    let count =
        usize::try_from(size * size - 1).map_err(|_| ValidationError::CapacityOverflow)?;
    let mut curves = solid::reserved(count)?;
    let mut previous = None;
    for distance in 0..size * size {
        let (x, y) = point(size, distance);
        // Cell centers keep the grid symmetrically inset from its usable bounds.
        let u = ((Real::from(x) + half.clone()) / &denominator)?;
        let v = ((Real::from(y) + half.clone()) / &denominator)?;
        let current = Point2::new(
            origin_x.clone() + width.clone() * u,
            origin_y.clone() + height.clone() * v,
        );
        if let Some(start) = previous {
            curves.push(Curve2::from(
                decisions.exact_curve(|| LineSeg2::try_new(start, current.clone()))?,
            ));
        }
        previous = Some(current);
    }
    let path = decisions.exact_curve(|| CurvePath2::try_new(curves))?;
    let paths = decisions.exact_curve(|| path.trim_inside_region(boundary))?;
    Ok(decisions.finish(paths))
}

/// Triangulates a filled region as a flat native triangle surface.
///
/// Certified linear profiles retain exact coordinates and hole ownership.
/// Other representations retain the existing finite sampled-profile path;
/// its predicate certainty does not certify fidelity to the source curves.
pub fn try_triangulate(
    input: &CurveRegion2,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let mesh = try_triangulate_with(input, &decisions)?;
    Ok(decisions.finish(mesh))
}

fn try_triangulate_with(
    input: &CurveRegion2,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for (material, holes) in try_modeling_profiles_with(input, decisions)? {
        let faces = decisions.consume_triangulation(hypercurve::triangulate_linear_contours(
            &decisions.triangulation_context(),
            &material,
            &holes,
        )?);
        for face in faces {
            let points = face.map(|p| Point3::new(p.x().clone(), p.y().clone(), Real::zero()));
            if !emit_certified_triangle(&mut positions, &mut triangles, points, false) {
                return Err(ValidationError::InvalidArguments);
            }
        }
    }
    validate_surface_mesh(TriangleMesh::new(positions, triangles), decisions)
}

/// Prefer the authoritative exact linear boundary before any finite projection.
/// Other representations retain the existing sampled-profile contract; the separate
/// finite projection remains a fidelity boundary, not proof of the source curve.
fn try_modeling_profiles_with(
    input: &CurveRegion2,
    decisions: &GeometryDecisions,
) -> Result<Vec<(Contour2, Vec<Contour2>)>, ValidationError> {
    let exact = decisions.exact_curve(|| {
        input
            .boundary_profiles()?
            .iter()
            .map(|profile| profile.linear_contours())
            .collect::<hypercurve::ExactCurveResult<Option<Vec<_>>>>()
    })?;
    if let Some(profiles) = exact {
        return Ok(profiles);
    }
    let quantized_contour = |ring: &FinitePolyline2| -> Result<Contour2, ValidationError> {
        let coordinates = ring
            .points()
            .iter()
            .map(|[x, y]| Ok([Real::try_from(*x)?, Real::try_from(*y)?]))
            .collect::<Result<Vec<_>, hyperreal::Problem>>()?;
        Contour2::from_real_ring(&coordinates).map_err(|cause| {
            ValidationError::ExactCurve(hypercurve::ExactCurveError::Invalid {
                operation: hypercurve::CurveOperation2::NativeTopology,
                family: Some(hypercurve::CurveFamily2::Line),
                cause,
            })
        })
    };
    try_finite_profiles_with(input, decisions)?
        .iter()
        .map(|profile| {
            let material = quantized_contour(profile.material())?;
            let holes = profile
                .holes()
                .iter()
                .map(quantized_contour)
                .collect::<Result<Vec<_>, _>>()?;
            Ok((material, holes))
        })
        .collect()
}

/// Finite polygon-with-holes boundary view for rendering and interchange.
pub fn try_finite_profiles(
    input: &CurveRegion2,
    context: &GeometryContext,
) -> Result<GeometryOutcome<Vec<FiniteRegionProfile2>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let profiles = try_finite_profiles_with(input, &decisions)?;
    Ok(decisions.finish(profiles))
}

fn try_finite_profiles_with(
    input: &CurveRegion2,
    decisions: &GeometryDecisions,
) -> Result<Vec<FiniteRegionProfile2>, ValidationError> {
    let options = FiniteProjectionOptions::try_new(1.0e-3)
        .expect("positive finite projection tolerance");
    let profiles =
        decisions.exact_curve(|| input.project_to_finite_profiles_exact(&options))?;
    Ok(profiles)
}

/// Linear extrusion that reports invalid or uncertain geometry.
///
/// Certified linear profiles keep their exact boundary coordinates. Other
/// representations use the existing finite sampled-profile path, with no
/// claim that predicate certainty certifies source-curve fidelity.
pub fn try_extrude(
    region: &CurveRegion2,
    height: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    try_extrude_vector(
        region,
        Vector3::from_xyz(Real::zero(), Real::zero(), height),
        context,
    )
}

/// Vector extrusion of a native filled region.
///
/// The direction must have a nonzero Z component. Profile fidelity has the
/// same contract as [`try_extrude`].
pub fn try_extrude_vector(
    region: &CurveRegion2,
    direction: Vector3,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let mesh = try_extrude_vector_with(region, direction, &decisions)?;
    Ok(decisions.finish(mesh))
}

fn try_extrude_vector_with(
    region: &CurveRegion2,
    direction: Vector3,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    let direction_sign = decisions.decide(
        hyperlimit::classify_real_sign(&direction.0[2], decisions.predicate_policy()),
        "extrusion direction Z sign",
    )?;
    if direction_sign == hyperlimit::Sign::Zero {
        return Err(ValidationError::ZeroParameter {
            name: "extrusion direction Z component",
        });
    }
    let flip = direction_sign == hyperlimit::Sign::Negative;
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let point = |p: &Point2| Point3::new(p.x().clone(), p.y().clone(), Real::zero());
    for (material, holes) in try_modeling_profiles_with(region, decisions)? {
        let cap_faces =
            decisions.consume_triangulation(hypercurve::triangulate_linear_contours(
                &decisions.triangulation_context(),
                &material,
                &holes,
            )?);
        for face in cap_faces {
            let [a, b, c] = face.map(|p| point(&p));
            if !emit_certified_triangle(
                &mut positions,
                &mut triangles,
                [a.clone(), b.clone(), c.clone()],
                !flip,
            ) || !emit_certified_triangle(
                &mut positions,
                &mut triangles,
                [a + &direction, b + &direction, c + &direction],
                flip,
            ) {
                return Err(ValidationError::InvalidArguments);
            }
        }
        for (ring, is_hole) in
            std::iter::once((&material, false)).chain(holes.iter().map(|ring| (ring, true)))
        {
            // Hypertri emits caps with a normalized positive outer winding and
            // negative hole winding. Exact curve-region projection preserves
            // authored loop orientation, so align each side ring with the cap
            // role instead of assuming every material loop arrived CCW.
            let reverse_ring = match contour_ring_sign(ring, decisions)? {
                hyperlimit::Sign::Positive => is_hole,
                hyperlimit::Sign::Negative => !is_hole,
                hyperlimit::Sign::Zero => {
                    return Err(ValidationError::Geometry(
                        "extrusion side ring has zero signed area".into(),
                    ));
                },
            };
            for edge in ring.segments() {
                let mut bottom_a = point(edge.start());
                let mut bottom_b = point(edge.end());
                if reverse_ring {
                    std::mem::swap(&mut bottom_a, &mut bottom_b);
                }
                let top_a = bottom_a.clone() + &direction;
                let top_b = bottom_b.clone() + &direction;
                if flip {
                    let emitted = emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [bottom_b.clone(), bottom_a, top_a.clone()],
                        false,
                    ) && emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [bottom_b, top_a, top_b],
                        false,
                    );
                    if !emitted {
                        return Err(ValidationError::InvalidArguments);
                    }
                } else {
                    let emitted = emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [bottom_a.clone(), bottom_b, top_b.clone()],
                        false,
                    ) && emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [bottom_a, top_b, top_a],
                        false,
                    );
                    if !emitted {
                        return Err(ValidationError::InvalidArguments);
                    }
                }
            }
        }
    }
    validate_closed_mesh(
        TriangleMesh::new(positions, triangles),
        region.is_empty(),
        decisions,
    )
}

fn validate_surface_mesh(
    mesh: TriangleMesh,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    let valid = decisions.consume_mesh(
        mesh.has_unique_nondegenerate_triangles(decisions.mesh_context())
            .map_err(|error| ValidationError::Geometry(error.to_string()))?,
    );
    if valid {
        Ok(mesh)
    } else {
        Err(ValidationError::Geometry(
            "triangulation emitted duplicate, invalid, or degenerate triangles".into(),
        ))
    }
}

fn validate_closed_mesh(
    mesh: TriangleMesh,
    allow_empty: bool,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    if allow_empty && mesh.triangles.is_empty() {
        return Ok(mesh);
    }
    let closed = decisions.consume_mesh(
        mesh.is_closed_manifold_geometry(decisions.mesh_context())
            .map_err(|error| ValidationError::Geometry(error.to_string()))?,
    );
    if closed {
        Ok(mesh)
    } else {
        Err(ValidationError::Geometry(
            "operation did not produce a unique nondegenerate closed manifold".into(),
        ))
    }
}

/// Emits geometry whose nondegeneracy follows from an exact source contract,
/// or whose caller performs one exact whole-mesh validation before returning.
fn emit_certified_triangle(
    positions: &mut Vec<Point3>,
    triangles: &mut Vec<hypermesh::Triangle>,
    points: [Point3; 3],
    reverse: bool,
) -> bool {
    debug_assert!(crate::hyper_math::htriangle_area2_is_nonzero(
        &points[0], &points[1], &points[2]
    ));
    let mut indices = [0; 3];
    for (slot, point) in indices.iter_mut().zip(points) {
        *slot = positions
            .iter()
            .rposition(|candidate| candidate == &point)
            .unwrap_or_else(|| {
                let index = positions.len();
                positions.push(point);
                index
            });
    }
    if indices[0] == indices[1] || indices[1] == indices[2] || indices[2] == indices[0] {
        return true;
    }
    triangles.push(if reverse {
        hypermesh::Triangle::new(indices[2], indices[1], indices[0])
    } else {
        hypermesh::Triangle::new(indices[0], indices[1], indices[2])
    });
    true
}

fn finite_point([x, y]: [f64; 2], z: Real) -> Option<Point3> {
    Some(Point3::new(
        Real::try_from(x).ok()?,
        Real::try_from(y).ok()?,
        z,
    ))
}

/// Surface of revolution from a native filled region.
pub fn revolve(
    region: &CurveRegion2,
    angle_degrees: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if segments < 2 {
        return Err(ValidationError::FieldLessThan {
            name: "segments",
            min: 2,
        });
    }
    let sign = decisions.decide(
        hyperlimit::classify_real_sign(&angle_degrees, decisions.predicate_policy()),
        "revolution angle sign",
    )?;
    let magnitude_order = decisions.decide(
        hyperlimit::compare_reals(
            &angle_degrees.clone().abs(),
            &Real::from(360_u16),
            decisions.predicate_policy(),
        ),
        "revolution angle magnitude",
    )?;
    let full = match (magnitude_order, sign) {
        (Ordering::Less, hyperlimit::Sign::Positive | hyperlimit::Sign::Negative) => false,
        (Ordering::Equal, hyperlimit::Sign::Positive | hyperlimit::Sign::Negative) => true,
        _ => return Err(ValidationError::InvalidArguments),
    };
    if full && segments < 3 {
        return Err(ValidationError::FieldLessThan {
            name: "segments",
            min: 3,
        });
    }
    let sample_count = if full {
        segments
    } else {
        segments
            .checked_add(1)
            .ok_or(ValidationError::InvalidArguments)?
    };
    let samples = (0..sample_count)
        .map(|index| {
            let fraction = (Real::from(index as u64) / Real::from(segments as u64)).ok()?;
            let radians =
                (angle_degrees.clone() * Real::pi() * fraction / Real::from(180_u16)).ok()?;
            Some((radians.clone().sin(), radians.cos()))
        })
        .collect::<Option<Vec<_>>>()
        .ok_or(ValidationError::InvalidArguments)?;
    let map = |point: [f64; 2], sample: &(Real, Real)| -> Option<Point3> {
        let radius = Real::try_from(point[0]).ok()?;
        Some(Point3::new(
            radius.clone() * sample.1.clone(),
            Real::try_from(point[1]).ok()?,
            radius * sample.0.clone(),
        ))
    };
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for profile in try_finite_profiles_with(region, &decisions)? {
        for ring in std::iter::once(profile.material()).chain(profile.holes().iter()) {
            if ring.points().iter().any(|point| point[0].is_sign_negative())
                && ring.points().iter().any(|point| point[0].is_sign_positive())
            {
                return Err(ValidationError::InvalidArguments);
            }
            if ring.points().windows(2).any(|edge| edge[0] == edge[1]) {
                return Err(ValidationError::InvalidArguments);
            }
            let radial_positive = ring.points().iter().any(|point| point[0] > 0.0);
            let reverse = (sign == hyperlimit::Sign::Positive) != radial_positive;
            let mut unique_rows = Vec::<([f64; 2], Vec<usize>)>::new();
            let mut point_rows = Vec::with_capacity(ring.points().len());
            for &point in ring.points() {
                if let Some((_, row)) =
                    unique_rows.iter().find(|(candidate, _)| *candidate == point)
                {
                    point_rows.push(row.clone());
                    continue;
                }
                let radius =
                    Real::try_from(point[0]).map_err(|_| ValidationError::InvalidArguments)?;
                let height =
                    Real::try_from(point[1]).map_err(|_| ValidationError::InvalidArguments)?;
                let row = if point[0] == 0.0 {
                    let index = positions.len();
                    positions.push(Point3::new(Real::zero(), height, Real::zero()));
                    vec![index; samples.len()]
                } else {
                    samples
                        .iter()
                        .map(|sample| {
                            let index = positions.len();
                            positions.push(Point3::new(
                                radius.clone() * sample.1.clone(),
                                height.clone(),
                                radius.clone() * sample.0.clone(),
                            ));
                            index
                        })
                        .collect()
                };
                unique_rows.push((point, row.clone()));
                point_rows.push(row);
            }
            for (edge_index, edge) in ring.points().windows(2).enumerate() {
                for slice in 0..segments {
                    let next = if full {
                        (slice + 1) % samples.len()
                    } else {
                        slice + 1
                    };
                    let a = point_rows[edge_index][slice];
                    let b = point_rows[edge_index + 1][slice];
                    let c = point_rows[edge_index + 1][next];
                    let d = point_rows[edge_index][next];
                    // Rotation maps an axis endpoint to the same point in both
                    // adjacent slices. OpenSCAD-style rotate_extrude omits that
                    // collapsed fan triangle while retaining its nondegenerate
                    // neighbor.
                    for (axis_endpoint, indices) in [
                        (edge[1][0] == 0.0, [a, b, c]),
                        (edge[0][0] == 0.0, [a, c, d]),
                    ] {
                        if axis_endpoint
                            || indices[0] == indices[1]
                            || indices[1] == indices[2]
                            || indices[2] == indices[0]
                        {
                            continue;
                        }
                        triangles.push(if reverse {
                            hypermesh::Triangle::new(indices[2], indices[1], indices[0])
                        } else {
                            hypermesh::Triangle::new(indices[0], indices[1], indices[2])
                        });
                    }
                }
            }
        }
        if !full {
            let cap_faces =
                decisions
                    .exact_curve(|| profile.triangulate())
                    .map_err(|error| {
                        ValidationError::Geometry(format!("revolution cap: {error}"))
                    })?;
            for face in cap_faces {
                let [Some(a), Some(b), Some(c)] = face.map(|point| map(point, &samples[0]))
                else {
                    return Err(ValidationError::InvalidArguments);
                };
                if !emit_certified_triangle(
                    &mut positions,
                    &mut triangles,
                    [a, b, c],
                    sign == hyperlimit::Sign::Positive,
                ) {
                    return Err(ValidationError::InvalidArguments);
                }
                let [Some(a), Some(b), Some(c)] = face
                    .map(|point| map(point, samples.last().expect("samples are nonempty")))
                else {
                    return Err(ValidationError::InvalidArguments);
                };
                if !emit_certified_triangle(
                    &mut positions,
                    &mut triangles,
                    [a, b, c],
                    sign == hyperlimit::Sign::Negative,
                ) {
                    return Err(ValidationError::InvalidArguments);
                }
            }
        }
    }
    let mesh = validate_closed_mesh(
        TriangleMesh::new(positions, triangles),
        region.is_empty(),
        &decisions,
    )?;
    Ok(decisions.finish(mesh))
}

/// Sweeps a native filled region along a 3D point path.
pub fn try_sweep(
    region: &CurveRegion2,
    path: &[Point3],
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let mut deduplicated = Vec::with_capacity(path.len());
    for point in path {
        let duplicate = match deduplicated.last() {
            Some(previous) => decisions.decide(
                hyperlimit::point3_equal(previous, point, decisions.predicate_policy()),
                "sweep adjacent-point equality",
            )?,
            None => false,
        };
        if !duplicate {
            deduplicated.push(point.clone());
        }
    }
    let path = deduplicated;
    if path.len() < 2 {
        return Err(ValidationError::InvalidArguments);
    }
    let closed = match path.first().zip(path.last()) {
        Some((first, last)) => decisions.decide(
            hyperlimit::point3_equal(first, last, decisions.predicate_policy()),
            "sweep endpoint equality",
        )?,
        None => return Err(ValidationError::InvalidArguments),
    };
    let slice_count = if closed { path.len() - 1 } else { path.len() };
    if slice_count < if closed { 3 } else { 2 } {
        return Err(ValidationError::InvalidArguments);
    }
    for index in 0..slice_count {
        for previous in &path[..index] {
            if decisions.decide(
                hyperlimit::point3_equal(previous, &path[index], decisions.predicate_policy()),
                "sweep repeated-point equality",
            )? {
                return Err(ValidationError::InvalidArguments);
            }
        }
    }
    let segment_count = if closed { slice_count } else { slice_count - 1 };
    let Some(segment_directions) = (0..segment_count)
        .map(|index| {
            (&path[(index + 1) % slice_count] - &path[index])
                .normalize_checked()
                .ok()
        })
        .collect::<Option<Vec<_>>>()
    else {
        return Err(ValidationError::InvalidArguments);
    };
    let Some(tangents) = (0..slice_count)
        .map(|index| {
            if !closed && index == 0 {
                Some(segment_directions[0].clone())
            } else if !closed && index + 1 == slice_count {
                Some(segment_directions[segment_count - 1].clone())
            } else {
                let incoming =
                    &segment_directions[(index + segment_count - 1) % segment_count];
                let outgoing = &segment_directions[index % segment_count];
                (incoming + outgoing).normalize_checked().ok()
            }
        })
        .collect::<Option<Vec<_>>>()
    else {
        return Err(ValidationError::InvalidArguments);
    };
    let Ok(mut orientation) = Matrix4::rotation_between_vectors(&Vector3::z(), &tangents[0])
    else {
        return Err(ValidationError::InvalidArguments);
    };
    let mut frames = Vec::with_capacity(slice_count);
    frames.push((orientation.clone(), path[0].clone()));
    for index in 1..slice_count {
        let Ok(transport) =
            Matrix4::rotation_between_vectors(&tangents[index - 1], &tangents[index])
        else {
            return Err(ValidationError::InvalidArguments);
        };
        orientation = transport * orientation;
        frames.push((orientation.clone(), path[index].clone()));
    }
    let map = |point: [f64; 2], frame: &(Matrix4, Point3)| -> Option<Point3> {
        let local = finite_point(point, Real::zero())?;
        frame
            .0
            .transform_point3(&local)
            .ok()
            .map(|point| point + frame.1.to_vector())
    };
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for profile in try_finite_profiles_with(region, &decisions)? {
        for ring in std::iter::once(profile.material()).chain(profile.holes().iter()) {
            for edge in ring.points().windows(2) {
                let segment_count = if closed { slice_count } else { slice_count - 1 };
                for slice in 0..segment_count {
                    let next = (slice + 1) % slice_count;
                    let [Some(a), Some(b), Some(c), Some(d)] = [
                        map(edge[0], &frames[slice]),
                        map(edge[1], &frames[slice]),
                        map(edge[1], &frames[next]),
                        map(edge[0], &frames[next]),
                    ] else {
                        return Err(ValidationError::InvalidArguments);
                    };
                    if !emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [a.clone(), b, c.clone()],
                        false,
                    ) || !emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [a, c, d],
                        false,
                    ) {
                        return Err(ValidationError::InvalidArguments);
                    }
                }
            }
        }
        if !closed {
            let cap_faces = decisions
                .exact_curve(|| profile.triangulate())
                .map_err(|error| ValidationError::Geometry(format!("sweep cap: {error}")))?;
            for face in cap_faces {
                for (frame, reverse) in [
                    (&frames[0], true),
                    (frames.last().expect("frames are nonempty"), false),
                ] {
                    let [Some(a), Some(b), Some(c)] = face.map(|point| map(point, frame))
                    else {
                        return Err(ValidationError::InvalidArguments);
                    };
                    if !emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [a, b, c],
                        reverse,
                    ) {
                        return Err(ValidationError::InvalidArguments);
                    }
                }
            }
        }
    }
    // Frame transport can collapse otherwise valid source geometry, so sweep
    // performs one policy-consistent whole-mesh certification before return.
    let mesh = validate_closed_mesh(
        TriangleMesh::new(positions, triangles),
        region.is_empty(),
        &decisions,
    )?;
    Ok(decisions.finish(mesh))
}

/// Extrudes while varying twist and terminal XY scale.
pub fn extrude_twisted(
    input: &CurveRegion2,
    height: Real,
    twist_degrees: Real,
    end_scale: [Real; 2],
    slices: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if slices < 1 {
        return Err(ValidationError::FieldLessThan {
            name: "slices",
            min: 1,
        });
    }
    let height_sign = decisions.decide(
        hyperlimit::classify_real_sign(&height, decisions.predicate_policy()),
        "twisted-extrusion height sign",
    )?;
    if height_sign == hyperlimit::Sign::Zero {
        return Err(ValidationError::InvalidArguments);
    }
    for scale in &end_scale {
        if decisions.decide(
            hyperlimit::classify_real_sign(scale, decisions.predicate_policy()),
            "twisted-extrusion scale sign",
        )? != hyperlimit::Sign::Positive
        {
            return Err(ValidationError::InvalidArguments);
        }
    }
    let parameters = (0..=slices)
        .map(|index| {
            let fraction = (Real::from(index as u64) / Real::from(slices as u64)).ok()?;
            let radians = (twist_degrees.clone() * fraction.clone() * Real::pi()
                / Real::from(180_u16))
            .ok()?;
            Some((
                radians.clone().sin(),
                radians.cos(),
                Real::one() + (end_scale[0].clone() - Real::one()) * fraction.clone(),
                Real::one() + (end_scale[1].clone() - Real::one()) * fraction.clone(),
                height.clone() * fraction,
            ))
        })
        .collect::<Option<Vec<_>>>()
        .ok_or(ValidationError::InvalidArguments)?;
    let map = |point: [f64; 2], parameter: &(Real, Real, Real, Real, Real)| {
        let x = Real::try_from(point[0]).ok()? * parameter.2.clone();
        let y = Real::try_from(point[1]).ok()? * parameter.3.clone();
        Some(Point3::new(
            x.clone() * parameter.1.clone() - y.clone() * parameter.0.clone(),
            x * parameter.0.clone() + y * parameter.1.clone(),
            parameter.4.clone(),
        ))
    };
    let reverse_sides = height_sign == hyperlimit::Sign::Negative;
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for profile in try_finite_profiles_with(input, &decisions)? {
        for ring in std::iter::once(profile.material()).chain(profile.holes().iter()) {
            if ring.points().windows(2).any(|edge| edge[0] == edge[1]) {
                return Err(ValidationError::InvalidArguments);
            }
            for edge in ring.points().windows(2) {
                for slice in 0..slices {
                    let [Some(a), Some(b), Some(c), Some(d)] = [
                        map(edge[0], &parameters[slice]),
                        map(edge[1], &parameters[slice]),
                        map(edge[1], &parameters[slice + 1]),
                        map(edge[0], &parameters[slice + 1]),
                    ] else {
                        return Err(ValidationError::InvalidArguments);
                    };
                    if !emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [a.clone(), b, c.clone()],
                        reverse_sides,
                    ) || !emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [a, c, d],
                        reverse_sides,
                    ) {
                        return Err(ValidationError::InvalidArguments);
                    }
                }
            }
        }
        let cap_faces = decisions
            .exact_curve(|| profile.triangulate())
            .map_err(|error| {
                ValidationError::Geometry(format!("twisted-extrusion cap: {error}"))
            })?;
        for face in cap_faces {
            for (parameter, reverse) in [
                (&parameters[0], !reverse_sides),
                (
                    parameters.last().expect("parameters are nonempty"),
                    reverse_sides,
                ),
            ] {
                let [Some(a), Some(b), Some(c)] = face.map(|point| map(point, parameter))
                else {
                    return Err(ValidationError::InvalidArguments);
                };
                if !emit_certified_triangle(&mut positions, &mut triangles, [a, b, c], reverse)
                {
                    return Err(ValidationError::InvalidArguments);
                }
            }
        }
    }
    let mesh = validate_closed_mesh(
        TriangleMesh::new(positions, triangles),
        input.is_empty(),
        &decisions,
    )?;
    Ok(decisions.finish(mesh))
}

/// Filled contours sampled from Hypersdf's normalized inverse-square field.
///
/// Positive radii/iso, nonnegative padding and dimensions of at least two are
/// required, including for empty inputs. Errors remain typed. The exact chord
/// proposal uses native marching squares and nesting-aware region admission;
/// its numerical certainty does not certify fidelity to the continuous field.
#[cfg(feature = "metaballs")]
pub fn metaballs(
    balls: &[(Point2, Real)],
    resolution: (usize, usize),
    iso_value: Real,
    padding: Real,
) -> Result<CurveRegion2, ValidationError> {
    metaballs_with_context(
        balls,
        resolution,
        iso_value,
        padding,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Planar metaball sampling with the selected extraction policy and certainty.
/// Field construction and conservative bounds remain strict.
#[cfg(feature = "metaballs")]
pub fn metaballs_with_context(
    balls: &[(Point2, Real)],
    resolution: (usize, usize),
    iso_value: Real,
    padding: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    super::metaballs::sample(balls, resolution, iso_value, padding, context)
}

/// Filled contours traced from a grayscale image.
#[cfg(feature = "image-io")]
pub fn from_image(
    image: &image::GrayImage,
    threshold: u8,
) -> Result<CurveRegion2, super::image::RasterTraceError> {
    super::image::try_from_image(image, threshold)
        .map(super::image::RasterTraceReport::into_region)
}

/// Filled TrueType glyph outlines.
#[cfg(feature = "truetype-text")]
pub fn truetype_text(text: &str, font_data: &[u8], scale: Real) -> CurveRegion2 {
    super::truetype::text_region(text, font_data, scale)
}

/// Hershey stroke geometry as native open curve strings in Cartesian coordinates.
///
/// Hershey's source records use screen-style coordinates with positive Y
/// pointing down. The returned geometry reflects that axis so positive Y
/// points up, consistently with the rest of the planar curve API.
#[cfg(feature = "hershey-text")]
pub fn hershey_strings(
    text: &str,
    font: &hypercurve::hershey::Font<'_>,
    size: Real,
) -> Vec<CurveString2> {
    let source_to_cartesian = hypercurve::Similarity2::try_from_real_affine(
        Real::one(),
        Real::zero(),
        Real::zero(),
        -Real::one(),
        Real::zero(),
        Real::zero(),
    )
    .expect("Y reflection is a nonsingular exact similarity");
    hypercurve::hershey::strings(text, font, size)
        .into_iter()
        .map(|string| {
            string
                .transform_similarity(&source_to_cartesian)
                .expect("Hershey line strings remain valid under Y reflection")
        })
        .collect()
}

/// Applies a homogeneous matrix whose restriction to the XY plane is affine.
///
/// The image must remain in the XY plane and have a constant nonzero
/// homogeneous weight. That weight is divided out, including negative weights.
/// Perspective and out-of-plane maps return typed unsupported errors. A zero
/// constant weight with a nonzero coordinate image returns an unbounded error;
/// an all-zero homogeneous image is invalid. Hypercurve validates the planar
/// map's nonsingularity and topology, including for empty inputs.
pub fn try_transformed(
    input: &CurveRegion2,
    matrix: &Matrix4,
) -> Result<CurveRegion2, ValidationError> {
    try_transformed_with_context(input, matrix, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Applies the same planar matrix contract with aggregate predicate certainty.
pub fn try_transformed_with_context(
    input: &CurveRegion2,
    matrix: &Matrix4,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    for value in [&matrix.0[3][0], &matrix.0[3][1]] {
        if decisions.sign(value, "planar homogeneous weight is constant")?
            != hyperreal::RealSign::Zero
        {
            return Err(ValidationError::UnsupportedTransform {
                reason: "perspective planar transforms require a projective curve operation",
            });
        }
    }
    for value in [&matrix.0[2][0], &matrix.0[2][1], &matrix.0[2][3]] {
        if decisions.sign(value, "transform preserves the XY plane")?
            != hyperreal::RealSign::Zero
        {
            return Err(ValidationError::UnsupportedTransform {
                reason: "the transform leaves the XY plane",
            });
        }
    }
    let weight = &matrix.0[3][3];
    if decisions.sign(weight, "planar homogeneous weight")? == hyperreal::RealSign::Zero {
        let mut nonzero_image = false;
        for value in [
            &matrix.0[0][0],
            &matrix.0[0][1],
            &matrix.0[0][3],
            &matrix.0[1][0],
            &matrix.0[1][1],
            &matrix.0[1][3],
        ] {
            nonzero_image |= decisions.sign(value, "homogeneous coordinate image")?
                != hyperreal::RealSign::Zero;
        }
        if !nonzero_image {
            // [0,0,0,0] is not a homogeneous point, including at infinity.
            return Err(ValidationError::InvalidArguments);
        }
        return Err(ValidationError::UnboundedTransform {
            reason: "the XY plane has zero homogeneous weight",
        });
    }
    // z=0 removes the third column. Normalize the remaining homogeneous
    // coordinates; do not duplicate Hypercurve's affine determinant proof.
    let [m00, m01, m10, m11, tx, ty] = [
        (&matrix.0[0][0] / weight)?,
        (&matrix.0[0][1] / weight)?,
        (&matrix.0[1][0] / weight)?,
        (&matrix.0[1][1] / weight)?,
        (&matrix.0[0][3] / weight)?,
        (&matrix.0[1][3] / weight)?,
    ];
    let region =
        decisions.exact_curve(|| input.transform_affine(&m00, &m01, &m10, &m11, &tx, &ty))?;
    Ok(decisions.finish(region))
}

/// Applies the same checked planar transform as [`try_transformed`].
pub fn transformed(
    input: &CurveRegion2,
    matrix: &Matrix4,
) -> Result<CurveRegion2, ValidationError> {
    try_transformed(input, matrix)
}

/// Applies an exact planar translation to a filled native region.
pub fn try_translated(
    input: &CurveRegion2,
    x: Real,
    y: Real,
) -> Result<CurveRegion2, ValidationError> {
    try_translated_with_context(input, x, y, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Applies the curve transform with the selected policy and aggregate certainty.
pub fn try_translated_with_context(
    input: &CurveRegion2,
    x: Real,
    y: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let region = decisions.exact_curve(|| {
        input.transform_affine(
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &x,
            &y,
        )
    })?;
    Ok(decisions.finish(region))
}

/// Translates a planar region, preserving native failures.
pub fn translated(
    input: &CurveRegion2,
    x: Real,
    y: Real,
) -> Result<CurveRegion2, ValidationError> {
    try_translated(input, x, y)
}

/// Rotates a filled native region counterclockwise about the origin in degrees.
pub fn try_rotated(
    input: &CurveRegion2,
    z_degrees: Real,
) -> Result<CurveRegion2, ValidationError> {
    try_rotated_with_context(input, z_degrees, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Applies the curve transform with the selected policy and aggregate certainty.
pub fn try_rotated_with_context(
    input: &CurveRegion2,
    z_degrees: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let radians = (z_degrees * Real::pi() / Real::from(180_u16)).expect("180 is nonzero");
    let cosine = radians.clone().cos();
    let sine = radians.sin();
    let region = decisions.exact_curve(|| {
        input.transform_affine(
            &cosine,
            &(-sine.clone()),
            &sine,
            &cosine,
            &Real::zero(),
            &Real::zero(),
        )
    })?;
    Ok(decisions.finish(region))
}

/// Rotates a planar region, preserving native failures.
pub fn rotated(
    input: &CurveRegion2,
    z_degrees: Real,
) -> Result<CurveRegion2, ValidationError> {
    try_rotated(input, z_degrees)
}

/// Scales a filled native region independently along the planar axes.
pub fn try_scaled(
    input: &CurveRegion2,
    x: Real,
    y: Real,
) -> Result<CurveRegion2, ValidationError> {
    try_scaled_with_context(input, x, y, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Applies the curve transform with the selected policy and aggregate certainty.
pub fn try_scaled_with_context(
    input: &CurveRegion2,
    x: Real,
    y: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let region = decisions.exact_curve(|| {
        input.transform_affine(
            &x,
            &Real::zero(),
            &Real::zero(),
            &y,
            &Real::zero(),
            &Real::zero(),
        )
    })?;
    Ok(decisions.finish(region))
}

/// Scales a planar region, preserving singular or undecided transform errors.
pub fn scaled(
    input: &CurveRegion2,
    x: Real,
    y: Real,
) -> Result<CurveRegion2, ValidationError> {
    try_scaled(input, x, y)
}

/// Classifies an exact XY point. Only a proved boundary returns `Ok(None)`;
/// undecided classifications and native failures return typed errors.
pub fn contains_xy(
    input: &CurveRegion2,
    x: Real,
    y: Real,
) -> Result<Option<bool>, ValidationError> {
    contains_xy_with_context(input, x, y, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Classifies a point with the selected policy. Boundaries return `None`;
/// undecided classifications retain their native error and context.
pub fn contains_xy_with_context(
    input: &CurveRegion2,
    x: Real,
    y: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<Option<bool>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if input.is_empty() {
        return Ok(decisions.finish(Some(false)));
    }
    let classification =
        decisions.exact_curve(|| input.classify_point(&Point2::new(x, y).into()))?;
    let inside = match classification {
        RegionPointLocation::Inside => Some(true),
        RegionPointLocation::Outside => Some(false),
        RegionPointLocation::Boundary => None,
    };
    Ok(decisions.finish(inside))
}

/// Returns the certified exact planar bounds, embedded in the XY plane.
pub fn try_bounding_box(input: &CurveRegion2) -> Result<Option<Aabb>, ValidationError> {
    try_bounding_box_with_context(input, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Computes curve bounds with the selected policy and aggregate certainty.
pub fn try_bounding_box_with_context(
    input: &CurveRegion2,
    context: &GeometryContext,
) -> Result<GeometryOutcome<Option<Aabb>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let bounds = decisions.exact_curve(|| input.bounds())?.map(|bounds| {
        Aabb::new(
            Point3::new(bounds.min_x().clone(), bounds.min_y().clone(), Real::zero()),
            Point3::new(bounds.max_x().clone(), bounds.max_y().clone(), Real::zero()),
        )
    });
    Ok(decisions.finish(bounds))
}

/// Returns exact planar bounds, absent for empty regions; errors remain typed.
pub fn bounding_box(input: &CurveRegion2) -> Result<Option<Aabb>, ValidationError> {
    try_bounding_box(input)
}

/// Returns the same native solid translation helper used by the 3D grammar.
pub fn translated_solid(
    mesh: &TriangleMesh,
    x: Real,
    y: Real,
    z: Real,
) -> Result<TriangleMesh, ValidationError> {
    solid::SolidExt::translated(mesh, x, y, z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_cache_keeps_only_strict_success_and_owned_identity() {
        REGION_RING_CACHE.with_borrow_mut(Vec::clear);
        for _ in 0..2 {
            assert!(polygon(&[]).is_err());
            assert!(square(terminally_unresolved_zero()).is_err());
            assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 0);
        }
        let unit = square(Real::one()).unwrap();
        assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 1);
        let warm =
            square_with_context(Real::one(), &GeometryContext::APPROXIMATE_512).unwrap();
        assert_eq!(warm.certainty, crate::GeometryCertainty::Certified);
        let scaled = try_scaled(&warm.value, Real::from(2), Real::one()).unwrap();
        assert_eq!(scaled.filled_area().unwrap(), Some(Real::from(2)));
        assert_eq!(unit.filled_area().unwrap(), Some(Real::one()));
        for width in 2..=66 {
            let region = rectangle(Real::from(width), Real::one()).unwrap();
            assert_eq!(region.filled_area().unwrap(), Some(Real::from(width)));
        }
        assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 64);
        assert_eq!(
            square(Real::one()).unwrap().filled_area().unwrap(),
            Some(Real::one())
        );
        REGION_RING_CACHE.with_borrow_mut(Vec::clear);
        let outcome =
            square_with_context(Real::one(), &GeometryContext::APPROXIMATE_512).unwrap();
        assert_eq!(outcome.certainty, crate::GeometryCertainty::Certified);
        assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 0);
        let child = std::thread::spawn(|| {
            assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 0);
            assert_eq!(
                square(Real::one()).unwrap().filled_area().unwrap(),
                Some(Real::one())
            );
        });
        child.join().unwrap();
        assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 0);
    }

    fn terminally_unresolved_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        &sine * &sine + &cosine * &cosine - Real::one()
    }

    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        mesh.triangles
            .iter()
            .filter_map(|triangle| {
                let a = mesh.positions[triangle.v0].to_vector();
                let b = mesh.positions[triangle.v1].to_vector();
                let c = mesh.positions[triangle.v2].to_vector();
                a.dot(&b.cross(&c)).to_f64_lossy()
            })
            .sum::<f64>()
            / 6.0
    }

    fn assert_exact_solid_boundary(mesh: &TriangleMesh, label: &str) {
        assert!(
            mesh.has_unique_nondegenerate_triangles(&crate::MESH_CONTEXT)
                .expect("boundary predicate")
                .into_value(),
            "{label} contains duplicate or degenerate exact triangles"
        );
        assert!(
            mesh.is_closed_manifold_geometry(&crate::MESH_CONTEXT)
                .expect("boundary predicate")
                .into_value(),
            "{label} is not an exact geometric two-manifold"
        );
    }

    #[test]
    fn finite_profiles_reuse_retained_loop_topology_without_reconsumption() {
        let start_x = Real::e().sin();
        let terminal_start_x = start_x.clone() + terminally_unresolved_zero();
        let p0 = Point2::new(start_x.clone(), Real::zero());
        let p1 = Point2::new(start_x.clone() + Real::one(), Real::zero());
        let p2 = Point2::new(start_x.clone() + Real::one(), Real::one());
        let p3 = Point2::new(start_x, Real::one());
        let terminal_p0 = Point2::new(terminal_start_x, Real::zero());
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p0, p1.clone()).unwrap()),
            Curve2::from(LineSeg2::try_new(p1, p2.clone()).unwrap()),
            Curve2::from(LineSeg2::try_new(p2, p3.clone()).unwrap()),
            Curve2::from(LineSeg2::try_new(p3, terminal_p0).unwrap()),
        ])
        .unwrap();
        let region = hypercurve::provisional(|| {
            CurveRegion2::try_from_boundary_paths(&[path], hypercurve::FillRule::EvenOdd)
        })
        .into_unverified()
        .unwrap();

        let strict = try_finite_profiles(&region, &GeometryContext::STRICT).unwrap();
        assert_eq!(strict.certainty, crate::GeometryCertainty::Certified);
        assert_eq!(strict.value.len(), 1);

        let permissive =
            try_finite_profiles(&region, &GeometryContext::APPROXIMATE_512).unwrap();
        assert_eq!(permissive.certainty, crate::GeometryCertainty::Certified);
        assert_eq!(permissive.value.len(), 1);
    }

    #[test]
    fn filled_constructor_and_extrusion_remain_native() {
        let region: CurveRegion2 = circle(Real::from(2_u8), 32).expect("valid planar fixture");
        assert!(!region.is_empty());
        let mesh: TriangleMesh =
            try_extrude(&region, Real::from(3_u8), &GeometryContext::STRICT)
                .expect("native extrusion")
                .into_value();
        assert!(!mesh.triangles.is_empty());
        assert!(mesh.is_closed_manifold());
        hypermesh::polygon_soup(&crate::MESH_CONTEXT, &[mesh.as_ref()])
            .expect("native extrusion must be reusable Hypermesh input");
    }

    #[test]
    fn profile_solid_constructors_share_closed_indexed_topology() {
        let square = square(Real::from(2_u8)).expect("valid planar fixture");
        let linear = try_extrude(&square, Real::from(2_u8), &GeometryContext::STRICT)
            .expect("linear extrusion")
            .into_value();
        assert!(linear.is_closed_manifold(), "linear extrusion");
        assert_exact_solid_boundary(&linear, "linear extrusion");
        assert!(signed_volume(&linear) > 0.0);
        let linear_negative =
            try_extrude(&square, Real::from(-2_i8), &GeometryContext::STRICT)
                .expect("negative linear extrusion")
                .into_value();
        assert!(
            linear_negative.is_closed_manifold(),
            "negative linear extrusion"
        );
        assert_exact_solid_boundary(&linear_negative, "negative linear extrusion");
        assert!(signed_volume(&linear_negative) > 0.0);
        let twisted = extrude_twisted(
            &square,
            Real::from(2_u8),
            Real::from(45_u8),
            [Real::one(), Real::one()],
            4,
            &GeometryContext::STRICT,
        )
        .expect("valid twisted extrusion")
        .into_value();
        assert!(twisted.is_closed_manifold(), "twisted extrusion");
        assert_exact_solid_boundary(&twisted, "twisted extrusion");
        assert!(signed_volume(&twisted) > 0.0);
        let path = [
            Point3::origin(),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
            Point3::new(Real::zero(), Real::zero(), Real::from(2_u8)),
        ];
        let swept = try_sweep(&square, &path, &GeometryContext::STRICT)
            .expect("valid sweep")
            .into_value();
        assert!(swept.is_closed_manifold(), "sweep");
        assert_exact_solid_boundary(&swept, "sweep");
        assert!(signed_volume(&swept) > 0.0);

        let radial = polygon(&[
            [Real::one(), -Real::one()],
            [Real::from(2_u8), -Real::one()],
            [Real::from(2_u8), Real::one()],
            [Real::one(), Real::one()],
        ])
        .expect("valid radial profile fixture");
        let revolved = revolve(&radial, Real::from(360_u16), 16, &GeometryContext::STRICT)
            .expect("valid revolution")
            .into_value();
        assert!(revolved.is_closed_manifold(), "revolution");
        assert_exact_solid_boundary(&revolved, "revolution");
        assert!(signed_volume(&revolved) > 0.0);
        for angle in [Real::from(180_u16), Real::from(-180_i16)] {
            let partial = revolve(&radial, angle, 8, &GeometryContext::STRICT)
                .expect("valid partial revolution")
                .into_value();
            assert!(partial.is_closed_manifold(), "partial revolution");
            assert_exact_solid_boundary(&partial, "partial revolution");
            assert!(signed_volume(&partial) > 0.0);
        }
    }

    #[test]
    fn sweep_rejects_revisited_open_path_vertices() {
        let square = square(Real::from(2_u8)).expect("valid planar fixture");
        let a = Point3::new(Real::from(-1_i8), Real::one(), Real::from(-1_i8));
        let b = Point3::new(Real::one(), Real::one(), Real::from(-1_i8));
        let path = [
            a.clone(),
            b.clone(),
            Point3::new(Real::one(), Real::from(-1_i8), Real::one()),
            Point3::new(Real::from(-1_i8), Real::one(), Real::one()),
            a,
            b,
        ];

        assert!(try_sweep(&square, &path, &GeometryContext::STRICT).is_err());
    }

    #[test]
    fn degenerate_profile_solid_requests_do_not_claim_solids() {
        let square = square(Real::from(2_u8)).expect("valid planar fixture");
        assert!(try_extrude(&square, Real::zero(), &GeometryContext::STRICT).is_err());
        assert!(
            try_extrude_vector(
                &square,
                Vector3::from_xyz(Real::one(), Real::zero(), Real::zero()),
                &GeometryContext::STRICT,
            )
            .is_err()
        );
        assert!(try_sweep(&square, &[Point3::origin()], &GeometryContext::STRICT).is_err());
        assert!(revolve(&square, Real::from(360_u16), 2, &GeometryContext::STRICT).is_err());
        assert!(
            revolve(
                &square,
                Real::from(180_u16),
                usize::MAX,
                &GeometryContext::STRICT,
            )
            .is_err()
        );
        assert!(
            extrude_twisted(
                &square,
                Real::one(),
                Real::zero(),
                [Real::zero(), Real::one()],
                2,
                &GeometryContext::STRICT,
            )
            .is_err()
        );
    }

    #[test]
    fn approximate_sweep_reports_consumed_terminal_point_equality() {
        let square = square(Real::from(2_u8)).expect("valid planar fixture");
        let symbolic_zero = terminally_unresolved_zero();
        let path = [
            Point3::origin(),
            Point3::new(symbolic_zero, Real::zero(), Real::zero()),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
        ];

        assert!(try_sweep(&square, &path, &GeometryContext::STRICT).is_err());
        let outcome = try_sweep(&square, &path, &GeometryContext::APPROXIMATE_512)
            .expect("approximate-512 should deduplicate the symbolic zero point");
        assert_eq!(
            outcome.certainty,
            crate::GeometryCertainty::Approximate512Consumed
        );
        assert!(outcome.value.is_closed_manifold());
    }

    #[test]
    fn native_region_boolean_does_not_construct_a_public_facade() {
        let left = square(Real::from(2_u8)).expect("valid planar fixture");
        let right =
            rectangle(Real::from(1_u8), Real::from(3_u8)).expect("valid planar fixture");
        let union: CurveRegion2 = left
            .boolean_region(&right, BooleanOp::Union)
            .expect("region union");
        assert!(!union.is_empty());
    }

    #[test]
    fn planar_constructors_reject_invalid_dimensions_and_ranges() {
        let negative = -Real::one();
        let invalid_compound =
            |result: Result<GeometryOutcome<CurveRegion2>, ValidationError>| result.is_err();
        assert!(circle(negative.clone(), 16).is_err());
        assert!(right_triangle(negative.clone(), Real::one()).is_err());
        assert!(ellipse(Real::one(), negative.clone(), 16).is_err());
        assert!(regular_ngon(5, negative.clone()).is_err());
        assert!(trapezoid(Real::one(), negative.clone(), Real::one(), Real::zero()).is_err());
        assert!(star(5, Real::one(), Real::from(2_u8)).is_err());
        assert!(invalid_compound(keyhole(
            Real::one(),
            Real::zero(),
            Real::one(),
            16,
            &GeometryContext::STRICT,
        )));
        assert!(invalid_compound(ring(
            Real::one(),
            negative.clone(),
            16,
            &GeometryContext::STRICT,
        )));
        assert!(invalid_compound(crescent(
            Real::one(),
            Real::from(2_u8),
            Real::zero(),
            16,
            &GeometryContext::STRICT,
        )));
        assert!(invalid_compound(circle_with_keyway(
            Real::one(),
            16,
            Real::from(2_u8),
            Real::one(),
            &GeometryContext::STRICT,
        )));
        assert!(invalid_compound(circle_with_flat(
            Real::one(),
            16,
            Real::one(),
            &GeometryContext::STRICT,
        )));
        assert!(invalid_compound(circle_with_two_flats(
            Real::one(),
            16,
            Real::one(),
            &GeometryContext::STRICT,
        )));
        assert!(
            airfoil_naca4(
                Real::from(10_u8),
                Real::from(4_u8),
                Real::from(12_u8),
                Real::one(),
                20,
            )
            .is_err()
        );
        assert!(
            airfoil_naca4(
                Real::from(2_u8),
                Real::from(10_u8),
                Real::from(12_u8),
                Real::one(),
                20,
            )
            .is_err()
        );
        assert!(
            airfoil_naca4(
                Real::zero(),
                Real::zero(),
                Real::from(100_u8),
                Real::one(),
                20,
            )
            .is_err()
        );
        assert!(star(usize::MAX, Real::from(2_u8), Real::one()).is_err());
        assert!(invalid_compound(reuleaux(
            usize::MAX,
            Real::one(),
            16,
            &GeometryContext::STRICT,
        )));
        assert!(teardrop(Real::one(), Real::from(2_u8), usize::MAX).is_err());
        assert!(
            involute_rack(
                Real::one(),
                usize::MAX,
                Real::from(20_u8),
                Real::zero(),
                Real::zero(),
            )
            .is_err()
        );
        assert!(cycloidal_rack(Real::one(), 1, Real::zero(), usize::MAX).is_err());
        assert!(
            cycloidal_gear(
                Real::one(),
                12,
                (Real::from(3_u8) / Real::from(4_u8)).unwrap(),
                Real::zero(),
                usize::MAX,
            )
            .is_err()
        );
        assert!(
            airfoil_naca4(
                Real::zero(),
                Real::zero(),
                Real::from(12_u8),
                Real::one(),
                usize::MAX,
            )
            .is_err()
        );
        assert!(bspline_path(&[], usize::MAX, 1).is_err());
    }

    #[test]
    fn normalized_egg_and_heart_match_requested_sampled_bounds() {
        let egg_bounds = bounding_box(
            &egg(Real::from(3_u8), Real::from(5_u8), 24)
                .expect("valid sampled profile fixture"),
        )
        .unwrap()
        .unwrap();
        assert_eq!(
            egg_bounds.maxs.x.clone() - egg_bounds.mins.x,
            Real::from(3_u8)
        );
        assert_eq!(
            egg_bounds.maxs.y.clone() - egg_bounds.mins.y,
            Real::from(5_u8)
        );

        let heart_bounds = bounding_box(
            &heart(Real::from(4_u8), Real::from(6_u8), 32)
                .expect("valid sampled profile fixture"),
        )
        .unwrap()
        .unwrap();
        assert_eq!(
            heart_bounds.maxs.x.clone() - heart_bounds.mins.x,
            Real::from(4_u8)
        );
        assert_eq!(
            heart_bounds.maxs.y.clone() - heart_bounds.mins.y,
            Real::from(6_u8)
        );
    }

    #[test]
    fn analytic_shape_parameters_change_their_geometry() {
        let involute_20 = involute_gear(
            Real::one(),
            16,
            Real::from(20_u8),
            Real::zero(),
            Real::zero(),
            4,
        )
        .expect("valid sampled mechanical fixture");
        let involute_25 = involute_gear(
            Real::one(),
            16,
            Real::from(25_u8),
            Real::zero(),
            Real::zero(),
            4,
        )
        .expect("valid sampled mechanical fixture");
        assert!(!involute_20.is_empty());
        assert!(!involute_25.is_empty());
        assert_ne!(involute_20, involute_25);

        let cycloidal_a = cycloidal_gear(Real::one(), 16, Real::one(), Real::zero(), 4)
            .expect("valid sampled mechanical fixture");
        let cycloidal_b = cycloidal_gear(
            Real::one(),
            16,
            (Real::from(5_u8) / Real::from(4_u8)).unwrap(),
            Real::zero(),
            4,
        )
        .expect("valid sampled mechanical fixture");
        assert!(!cycloidal_a.is_empty());
        assert!(!cycloidal_b.is_empty());
        assert_ne!(cycloidal_a, cycloidal_b);

        let rack_20 =
            involute_rack(Real::one(), 4, Real::from(20_u8), Real::zero(), Real::zero())
                .expect("valid sampled mechanical fixture");
        let rack_25 =
            involute_rack(Real::one(), 4, Real::from(25_u8), Real::zero(), Real::zero())
                .expect("valid sampled mechanical fixture");
        assert!(!rack_20.is_empty());
        assert!(!rack_25.is_empty());
        assert_ne!(rack_20, rack_25);
        assert_ne!(
            cycloidal_rack(Real::one(), 4, Real::zero(), 4)
                .expect("valid sampled mechanical fixture"),
            cycloidal_rack(Real::one(), 4, Real::zero(), 8)
                .expect("valid sampled mechanical fixture")
        );

        let symmetric =
            airfoil_naca4(Real::zero(), Real::zero(), Real::from(12_u8), Real::one(), 20)
                .expect("valid sampled mechanical fixture");
        let cambered = airfoil_naca4(
            Real::from(2_u8),
            Real::from(4_u8),
            Real::from(12_u8),
            Real::one(),
            20,
        )
        .expect("valid sampled mechanical fixture");
        assert!(!symmetric.is_empty());
        assert!(!cambered.is_empty());
        assert_ne!(symmetric, cambered);
    }

    #[test]
    fn cycloidal_rack_alternates_face_and_root_flanks() {
        let teeth = 4;
        let tip = Real::one();
        let pitch_line = Real::zero();
        let root = -(Real::from(5_u8) / Real::from(4_u8)).expect("four is nonzero");
        let rack = cycloidal_rack(Real::one(), teeth, Real::zero(), 8)
            .expect("valid sampled mechanical fixture");
        let paths = rack.boundary_paths().expect("cycloidal rack exact boundary");
        let paths = paths;
        assert_eq!(paths.len(), 1, "rack must have one closed boundary");

        let curves = paths[0].curves();
        let vertices_at = |level: &Real| {
            let witness =
                hypercurve::CurvePoint2::from(Point2::new(Real::zero(), level.clone()));
            curves
                .iter()
                .filter(|curve| {
                    match curve
                        .start()
                        .compare_coordinate(&witness, hypercurve::Axis2::Y)
                    {
                        Ok(order) => order == Ordering::Equal,
                        Err(hypercurve::ExactCurveError::Blocked(_)) => false,
                        Err(error) => panic!("finite rack coordinate order: {error:?}"),
                    }
                })
                .count()
        };
        // The neighboring root/face samples are scalar multiples of the
        // same (run, rise), so native admission may remove their collinear
        // pitch vertex. Count geometric crossings, independent of that split.
        let pitch = hypercurve::CurvePoint2::from(Point2::new(Real::zero(), pitch_line));
        let crossings = curves
            .iter()
            .filter(|curve| {
                let start = curve
                    .start()
                    .compare_coordinate(&pitch, hypercurve::Axis2::Y)
                    .expect("rack start height");
                let end = curve
                    .end()
                    .compare_coordinate(&pitch, hypercurve::Axis2::Y)
                    .expect("rack end height");
                (start == Ordering::Less) != (end == Ordering::Less)
            })
            .count();
        assert_eq!(crossings, teeth * 2, "both sides must cross the pitch line");
        let epsilon = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
        for tooth in 0..teeth {
            for side in [-1, 1] {
                let x = Real::pi()
                    * (Real::from(tooth as u64) + (Real::from(side) / Real::from(4)).unwrap());
                for (y, expected) in [
                    (Real::zero(), None),
                    (-&epsilon, Some(true)),
                    (epsilon.clone(), Some(false)),
                ] {
                    let location = contains_xy_with_context(
                        &rack,
                        x.clone(),
                        y,
                        &GeometryContext::STRICT,
                    )
                    .expect("exact rack pitch contact and displaced membership");
                    assert_eq!(location.certainty, crate::GeometryCertainty::Certified);
                    assert_eq!(location.value, expected);
                }
            }
        }
        assert_eq!(
            vertices_at(&tip),
            teeth * 2,
            "both addendum cycloids must reach every tooth tip"
        );
        assert!(
            vertices_at(&root) >= teeth * 2,
            "both dedendum cycloids must reach every tooth root"
        );
    }

    #[test]
    fn gear_and_rack_profiles_form_one_hole_free_material_component() {
        let cycloidal = cycloidal_gear(
            Real::one(),
            16,
            (Real::from(5_u8) / Real::from(4_u8)).unwrap(),
            Real::zero(),
            8,
        )
        .expect("valid sampled mechanical fixture");
        let involute =
            involute_rack(Real::one(), 6, Real::from(20_u8), Real::zero(), Real::zero())
                .expect("valid sampled mechanical fixture");
        let cycloidal_rack_profile = cycloidal_rack(Real::one(), 6, Real::zero(), 12)
            .expect("valid sampled mechanical fixture");
        let readme_cycloidal = cycloidal_gear(
            Real::try_from(0.22).unwrap(),
            14,
            Real::try_from(0.275).unwrap(),
            Real::try_from(0.02).unwrap(),
            8,
        )
        .expect("valid sampled mechanical fixture");
        assert!(
            cycloidal_gear(
                Real::one(),
                16,
                (Real::from(3_u8) / Real::from(4_u8)).unwrap(),
                Real::zero(),
                8,
            )
            .is_err(),
            "overlapping neighboring cycloidal root flanks must be rejected"
        );
        let projection =
            FiniteProjectionOptions::try_new(1.0e-3).expect("positive projection tolerance");

        for (name, region) in [
            ("cycloidal gear", cycloidal),
            ("README cycloidal gear", readme_cycloidal),
            ("involute rack", involute),
            ("cycloidal rack", cycloidal_rack_profile),
        ] {
            let profiles = region
                .project_to_finite_profiles_exact(&projection)
                .unwrap_or_else(|error| panic!("{name} profile projection failed: {error}"));
            assert_eq!(
                profiles.len(),
                1,
                "{name} must be one connected material component"
            );
            assert!(
                profiles[0].holes().is_empty(),
                "{name} must not acquire holes from a self-intersecting boundary"
            );
            let paths = region
                .boundary_paths()
                .unwrap_or_else(|error| panic!("{name} exact boundary failed: {error}"));
            let paths = paths;
            assert_eq!(
                paths.len(),
                1,
                "{name} must expose one simple closed edge path"
            );
        }

        for (name, rack) in [
            (
                "involute rack",
                involute_rack(Real::one(), 6, Real::from(20_u8), Real::zero(), Real::zero())
                    .expect("valid sampled mechanical fixture"),
            ),
            (
                "cycloidal rack",
                cycloidal_rack(Real::one(), 6, Real::zero(), 12)
                    .expect("valid sampled mechanical fixture"),
            ),
        ] {
            let bounds = try_bounding_box(&rack)
                .unwrap_or_else(|error| panic!("{name} bounds failed: {error}"))
                .expect("nonempty rack bounds");
            assert!(
                bounds.mins.y < -Real::one(),
                "{name} must include backing below its tooth-root line"
            );
        }
    }

    #[test]
    fn higher_order_curve_constructors_and_flat_triangulation_are_usable() {
        let zero = Real::zero();
        let one = Real::one();
        let two = Real::from(2_u8);
        let controls = [
            [zero.clone(), zero.clone()],
            [one.clone(), two.clone()],
            [two.clone(), one.clone()],
            [Real::from(3_u8), zero.clone()],
        ];
        assert!(bezier_path(&controls, 16).is_ok());
        assert!(bspline_path(&controls, 2, 8).is_ok());
        assert!(bezier_path(&controls, 0).is_err());
        assert!(bspline_path(&controls, 2, 0).is_err());
        assert!(matches!(
            bezier_region_with_context(&controls, 16, &GeometryContext::STRICT),
            Err(ValidationError::InvalidArguments)
        ));

        let closed_controls = [
            [zero.clone(), zero.clone()],
            [two.clone(), zero.clone()],
            [two.clone(), two.clone()],
            [zero.clone(), zero],
        ];
        assert!(!bezier_region(&closed_controls, 16).unwrap().is_empty());

        let mesh = try_triangulate(
            &square(two).expect("valid planar fixture"),
            &GeometryContext::STRICT,
        )
        .expect("square triangulation")
        .into_value();
        assert_eq!(mesh.positions.len(), 4);
        assert_eq!(mesh.triangles.len(), 2);
        assert!(
            mesh.triangles
                .iter()
                .flat_map(|triangle| [triangle.v0, triangle.v1, triangle.v2])
                .all(|index| index < mesh.positions.len())
        );
    }

    #[test]
    fn nominal_planar_shape_catalog_constructs_material() {
        let one = Real::one();
        let two = Real::from(2_u8);
        let three_quarters = (Real::from(3_u8) / Real::from(4_u8)).unwrap();
        let compound = |result: Result<GeometryOutcome<CurveRegion2>, ValidationError>| {
            result.expect("nominal compound region").into_value()
        };
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            let crescent = crescent(two.clone(), one.clone(), one.clone(), 16, &context)
                .expect("the symbolic crescent now has a strict scalar proof");
            assert_eq!(crescent.certainty, crate::GeometryCertainty::Certified);
            assert!(!crescent.value.is_empty());
            let profiles = try_finite_profiles(&crescent.value, &GeometryContext::STRICT)
                .expect("the certified crescent retains projectable boundary topology");
            assert_eq!(profiles.certainty, crate::GeometryCertainty::Certified);
            assert!(!profiles.value.is_empty());
            // (-1,0) is material, (1,0) lies in the removed unit polygon,
            // and (3,0) lies outside the outer radius-two polygon.
            for (x, expected) in [(-1, true), (1, false), (3, false)] {
                let location = contains_xy_with_context(
                    &crescent.value,
                    Real::from(x),
                    Real::zero(),
                    &GeometryContext::STRICT,
                )
                .expect("certified crescent membership");
                assert_eq!(location.certainty, crate::GeometryCertainty::Certified);
                assert_eq!(location.value, Some(expected));
            }
        }
        let shapes = [
            rectangle(two.clone(), one.clone()).expect("valid planar fixture"),
            square(two.clone()).expect("valid planar fixture"),
            circle(one.clone(), 16).expect("valid planar fixture"),
            right_triangle(two.clone(), one.clone()).expect("valid planar fixture"),
            polygon(&[
                [Real::zero(), Real::zero()],
                [one.clone(), Real::zero()],
                [Real::zero(), one.clone()],
            ])
            .expect("valid planar fixture"),
            ellipse(two.clone(), one.clone(), 16).expect("valid planar fixture"),
            regular_ngon(5, one.clone()).expect("valid planar fixture"),
            arrow(two.clone(), one.clone(), one.clone(), two.clone())
                .expect("valid sampled profile fixture"),
            trapezoid(one.clone(), two.clone(), one.clone(), one.clone())
                .expect("valid sampled profile fixture"),
            star(5, two.clone(), one.clone()).expect("valid sampled profile fixture"),
            teardrop(two.clone(), Real::from(3_u8), 12)
                .expect("valid sampled profile fixture"),
            egg(two.clone(), Real::from(3_u8), 16).expect("valid sampled profile fixture"),
            rounded_rectangle(two.clone(), one.clone(), three_quarters.clone(), 4)
                .expect("valid planar fixture"),
            squircle(two.clone(), one.clone(), 16).expect("valid sampled profile fixture"),
            compound(keyhole(
                one.clone(),
                one.clone(),
                two.clone(),
                16,
                &GeometryContext::APPROXIMATE_512,
            )),
            compound(reuleaux(
                3,
                two.clone(),
                24,
                &GeometryContext::APPROXIMATE_512,
            )),
            compound(ring(
                one.clone(),
                three_quarters.clone(),
                16,
                &GeometryContext::APPROXIMATE_512,
            )),
            pie_slice(one.clone(), Real::zero(), Real::from(90_u8), 8)
                .expect("valid sampled profile fixture"),
            heart(two.clone(), two.clone(), 16).expect("valid sampled profile fixture"),
            supershape(
                one.clone(),
                one.clone(),
                Real::from(5_u8),
                two.clone(),
                one.clone(),
                one.clone(),
                24,
            )
            .expect("valid sampled profile fixture"),
            compound(circle_with_keyway(
                two.clone(),
                24,
                one.clone(),
                three_quarters.clone(),
                &GeometryContext::APPROXIMATE_512,
            )),
            compound(circle_with_flat(
                two.clone(),
                24,
                one.clone(),
                &GeometryContext::APPROXIMATE_512,
            )),
            compound(circle_with_two_flats(
                two.clone(),
                24,
                one.clone(),
                &GeometryContext::APPROXIMATE_512,
            )),
            involute_gear(
                one.clone(),
                16,
                Real::from(20_u8),
                Real::zero(),
                Real::zero(),
                4,
            )
            .expect("valid sampled mechanical fixture"),
            cycloidal_gear(one.clone(), 16, one.clone(), Real::zero(), 4)
                .expect("valid sampled mechanical fixture"),
            involute_rack(one.clone(), 4, Real::from(20_u8), Real::zero(), Real::zero())
                .expect("valid sampled mechanical fixture"),
            cycloidal_rack(one.clone(), 4, Real::zero(), 8)
                .expect("valid sampled mechanical fixture"),
            airfoil_naca4(Real::from(2_u8), Real::from(4_u8), Real::from(12_u8), one, 20)
                .expect("valid sampled mechanical fixture"),
        ];
        for (index, shape) in shapes.into_iter().enumerate() {
            assert!(!shape.is_empty(), "planar catalog entry {index}");
            assert!(
                !try_finite_profiles(&shape, &GeometryContext::APPROXIMATE_512)
                    .unwrap_or_else(|error| {
                        panic!("finite catalog profile {index} failed: {error}")
                    })
                    .into_value()
                    .is_empty(),
                "finite profile for catalog entry {index}"
            );
        }
    }

    #[test]
    fn hilbert_samples_are_centered_symmetrically_in_the_usable_grid() {
        let strings = hilbert_strings(
            &square(Real::from(8_u8)).expect("valid planar fixture"),
            3,
            Real::one(),
        )
        .expect("the square has an exact Hilbert line string");
        let points = strings
            .iter()
            .flat_map(|string| string.segments())
            .flat_map(|segment| [segment.start(), segment.end()])
            .collect::<Vec<_>>();
        assert!(!points.is_empty());
        let order = |left: &&Real, right: &&Real| {
            hyperlimit::compare_reals(left, right, crate::PREDICATE_POLICY)
                .value()
                .expect(
                    "catalog coordinates must be ordered by the centralized predicate policy",
                )
        };
        let min_x = points.iter().map(|point| point.x()).min_by(order).unwrap();
        let max_x = points.iter().map(|point| point.x()).max_by(order).unwrap();
        let min_y = points.iter().map(|point| point.y()).min_by(order).unwrap();
        let max_y = points.iter().map(|point| point.y()).max_by(order).unwrap();
        let expected_min = (Real::from(11_u8) / Real::from(8_u8)).unwrap();
        let expected_max = (Real::from(53_u8) / Real::from(8_u8)).unwrap();
        assert_eq!(min_x, &expected_min);
        assert_eq!(min_y, &expected_min);
        assert_eq!(max_x, &expected_max);
        assert_eq!(max_y, &expected_max);
    }

    #[cfg(feature = "hershey-text")]
    #[test]
    fn hershey_catalog_is_native_hypercurve_code() {
        let strings =
            hershey_strings("CSG", &crate::curve::hershey::fonts::FUTURAL, Real::one());

        assert!(!strings.is_empty());
        assert_eq!(crate::curve::hershey::fonts::ALL.len(), 32);
        assert!(strings.iter().all(|string| !string.segments().is_empty()));
    }

    #[cfg(feature = "hershey-text")]
    #[test]
    fn hershey_strings_convert_source_coordinates_to_y_up() {
        let font = crate::curve::hershey::Font::new(&["MWRMNV RRMVV"], 'A');
        let strings = hershey_strings("A", &font, Real::one());

        assert_eq!(strings.len(), 2);
        let left = &strings[0].segments()[0];
        let right = &strings[1].segments()[0];
        assert_eq!(left.start(), &Point2::new(Real::zero(), Real::from(5_u8)));
        assert_eq!(left.end(), &Point2::new(Real::from(-4_i8), Real::from(-4_i8)));
        assert_eq!(right.start(), &Point2::new(Real::zero(), Real::from(5_u8)));
        assert_eq!(right.end(), &Point2::new(Real::from(4_u8), Real::from(-4_i8)));
    }
}
