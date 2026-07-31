//! CSG grammar over native Hypercurve carriers.
//!
//! Filled construction returns [`CurveRegion2`]. Open geometry is returned as
//! [`CurveString2`] or [`CurvePath2`] by the APIs that create it; csgrs does not
//! combine those distinct native meanings in a public profile facade.

use crate::context::GeometryDecisions;
use crate::errors::{CurveBooleanError, ValidationError};
use crate::solid;
use crate::{GeometryContext, GeometryOutcome};
use hypercurve::{
    BooleanOp, Classification, Contour2, CubicBezier2, Curve2, CurveOutcome, CurvePath2,
    CurvePolicy, CurveRegion2, CurveString2, ExactCurveResult, FinitePolyline2,
    FiniteProjectionOptions, FiniteRegionProfile2, LineSeg2, Point2, PolynomialSplineCurve2,
    QuadraticBezier2, RationalBezier2, RegionPointLocation,
};
use hyperlattice::{Aabb, Matrix4, Point3, Real, Vector3};
use hypermesh::TriangleMesh;
use std::cell::RefCell;
use std::cmp::Ordering;

thread_local! {
    static REGION_RING_CACHE: RefCell<Vec<(Vec<[Real; 2]>, CurveRegion2)>> =
        const { RefCell::new(Vec::new()) };
}

fn region_from_ring(points: &[[Real; 2]]) -> CurveRegion2 {
    if let Some(region) = REGION_RING_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .rev()
            .find(|(cached_points, _)| cached_points == points)
            .map(|(_, region)| region.clone())
    }) {
        return region;
    }
    let Ok(contour) = Contour2::from_real_ring(points) else {
        return CurveRegion2::empty();
    };
    let region =
        CurveRegion2::try_from_native_material_contours(vec![contour], &CurvePolicy::STRICT)
            .unwrap_or_else(|_| CurveRegion2::empty());
    REGION_RING_CACHE.with_borrow_mut(|entries| {
        const CAPACITY: usize = 64;
        if entries.len() == CAPACITY {
            entries.remove(0);
        }
        entries.push((points.to_vec(), region.clone()));
    });
    region
}

fn sampled_ellipse(radius_x: Real, radius_y: Real, segments: usize) -> CurveRegion2 {
    if segments < 3 || !positive_real(&radius_x) || !positive_real(&radius_y) {
        return CurveRegion2::empty();
    }
    let denominator = Real::from(segments as u64);
    let points = (0..segments)
        .map(|index| {
            let fraction = (Real::from(index as u64) / &denominator).ok()?;
            let angle = Real::tau() * fraction;
            Some([
                radius_x.clone() * angle.clone().cos(),
                radius_y.clone() * angle.sin(),
            ])
        })
        .collect::<Option<Vec<_>>>();
    points.map_or_else(CurveRegion2::empty, |points| region_from_ring(&points))
}

fn positive_real(value: &Real) -> bool {
    matches!(
        crate::hyper_math::hreal_sign(value),
        Some(hyperreal::RealSign::Positive)
    )
}

fn nonnegative_real(value: &Real) -> bool {
    matches!(
        crate::hyper_math::hreal_sign(value),
        Some(hyperreal::RealSign::Positive | hyperreal::RealSign::Zero)
    )
}

fn real_cmp(left: &Real, right: &Real) -> Option<Ordering> {
    crate::hyper_math::hreal_try_cmp(left, right)
}

fn points_equal(left: &Point2, right: &Point2) -> Option<bool> {
    Some(
        real_cmp(left.x(), right.x())? == Ordering::Equal
            && real_cmp(left.y(), right.y())? == Ordering::Equal,
    )
}

fn finite_ring_sign(
    ring: &FinitePolyline2,
    decisions: &GeometryDecisions,
) -> Result<hyperlimit::Sign, ValidationError> {
    let points = ring
        .points()
        .iter()
        .map(|[x, y]| {
            Some(hyperlimit::Point2::new(
                Real::try_from(*x).ok()?,
                Real::try_from(*y).ok()?,
            ))
        })
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| {
            ValidationError::Geometry(
                "finite side ring contains a non-finite projected coordinate".into(),
            )
        })?;
    decisions.decide(
        hyperlimit::ring_area_sign(&points, decisions.predicate_policy()),
        "extrusion side-ring signed area",
    )
}

fn exact_ratio(numerator: usize, denominator: usize) -> Option<Real> {
    (Real::from(numerator as u64) / Real::from(denominator as u64)).ok()
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
) -> Option<Vec<[Real; 2]>> {
    let capacity = copies.checked_mul(local_points.len())?;
    let mut points = Vec::with_capacity(capacity);
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
    Some(points)
}

/// Empty filled curve topology.
pub fn empty() -> CurveRegion2 {
    CurveRegion2::empty()
}

/// Axis-aligned rectangle with its minimum corner at the origin.
pub fn rectangle(width: Real, length: Real) -> CurveRegion2 {
    if !matches!(
        crate::hyper_math::hreal_sign(&width),
        Some(hyperreal::RealSign::Positive)
    ) || !matches!(
        crate::hyper_math::hreal_sign(&length),
        Some(hyperreal::RealSign::Positive)
    ) {
        return empty();
    }
    region_from_ring(&[
        [Real::zero(), Real::zero()],
        [width.clone(), Real::zero()],
        [width, length.clone()],
        [Real::zero(), length],
    ])
}

/// Axis-aligned square with its minimum corner at the origin.
pub fn square(width: Real) -> CurveRegion2 {
    rectangle(width.clone(), width)
}

/// Regular sampled circle.
pub fn circle(radius: Real, segments: usize) -> CurveRegion2 {
    sampled_ellipse(radius.clone(), radius, segments)
}

/// Right triangle with its right-angle corner at the origin.
pub fn right_triangle(width: Real, height: Real) -> CurveRegion2 {
    if !positive_real(&width) || !positive_real(&height) {
        return empty();
    }
    region_from_ring(&[
        [Real::zero(), Real::zero()],
        [width, Real::zero()],
        [Real::zero(), height],
    ])
}

/// Closed polygon from exact coordinate rows.
pub fn polygon(points: &[[Real; 2]]) -> CurveRegion2 {
    region_from_ring(points)
}

/// Closed polygon from native Hypercurve points.
pub fn polygon_points(points: &[Point2]) -> CurveRegion2 {
    let points = points
        .iter()
        .map(|point| [point.x().clone(), point.y().clone()])
        .collect::<Vec<_>>();
    region_from_ring(&points)
}

/// Sampled ellipse.
pub fn ellipse(width: Real, height: Real, segments: usize) -> CurveRegion2 {
    if !positive_real(&width) || !positive_real(&height) {
        return empty();
    }
    let two = Real::from(2_u8);
    let (Ok(radius_x), Ok(radius_y)) = (width / &two, height / &two) else {
        return empty();
    };
    sampled_ellipse(radius_x, radius_y, segments)
}

/// Regular polygon.
pub fn regular_ngon(sides: usize, radius: Real) -> CurveRegion2 {
    sampled_ellipse(radius.clone(), radius, sides)
}

/// Filled arrow.
pub fn arrow(
    shaft_length: Real,
    shaft_width: Real,
    head_length: Real,
    head_width: Real,
) -> CurveRegion2 {
    if [
        shaft_length.clone(),
        shaft_width.clone(),
        head_length.clone(),
        head_width.clone(),
    ]
    .iter()
    .any(|value| {
        !matches!(
            crate::hyper_math::hreal_sign(value),
            Some(hyperreal::RealSign::Positive)
        )
    }) {
        return empty();
    }
    let two = Real::from(2_u8);
    let half_shaft = (shaft_width / &two).expect("two is nonzero");
    let half_head = (head_width / &two).expect("two is nonzero");
    let tip = shaft_length.clone() + head_length;
    region_from_ring(&[
        [Real::zero(), -half_shaft.clone()],
        [shaft_length.clone(), -half_shaft.clone()],
        [shaft_length.clone(), -half_head.clone()],
        [tip, Real::zero()],
        [shaft_length.clone(), half_head],
        [shaft_length, half_shaft.clone()],
        [Real::zero(), half_shaft],
    ])
}

/// Filled trapezoid.
pub fn trapezoid(
    top_width: Real,
    bottom_width: Real,
    height: Real,
    top_offset: Real,
) -> CurveRegion2 {
    if !positive_real(&top_width) || !positive_real(&bottom_width) || !positive_real(&height) {
        return empty();
    }
    region_from_ring(&[
        [Real::zero(), Real::zero()],
        [bottom_width, Real::zero()],
        [top_offset.clone() + top_width, height.clone()],
        [top_offset, height],
    ])
}

/// Alternating-radius star polygon.
pub fn star(num_points: usize, outer_radius: Real, inner_radius: Real) -> CurveRegion2 {
    let Some(count) = num_points.checked_mul(2) else {
        return empty();
    };
    if num_points < 3
        || !positive_real(&inner_radius)
        || real_cmp(&outer_radius, &inner_radius) != Some(Ordering::Greater)
    {
        return empty();
    }
    let denominator = Real::from(count as u64);
    let points = (0..count)
        .filter_map(|index| {
            let angle = Real::tau() * (Real::from(index as u64) / &denominator).ok()?;
            let radius = if index % 2 == 0 {
                &outer_radius
            } else {
                &inner_radius
            };
            Some([
                radius.clone() * angle.clone().cos(),
                radius.clone() * angle.sin(),
            ])
        })
        .collect::<Vec<_>>();
    if points.len() == count {
        region_from_ring(&points)
    } else {
        empty()
    }
}

/// Teardrop region.
pub fn teardrop(width: Real, length: Real, segments: usize) -> CurveRegion2 {
    if segments < 2 || !positive_real(&width) || !positive_real(&length) {
        return empty();
    }
    let Some(point_capacity) = segments.checked_add(2) else {
        return empty();
    };
    let Some(radius) = (width / Real::from(2_u8))
        .ok()
        .filter(|radius| real_cmp(&length, radius) == Some(Ordering::Greater))
    else {
        return empty();
    };
    let center_y = length - radius.clone();
    let mut points = Vec::with_capacity(point_capacity);
    points.push([Real::zero(), Real::zero()]);
    for index in 0..=segments {
        let Some(fraction) = (Real::from(index as u64) / Real::from(segments as u64)).ok()
        else {
            return empty();
        };
        let angle = Real::pi() * fraction;
        points.push([
            -radius.clone() * angle.clone().cos(),
            center_y.clone() + radius.clone() * angle.sin(),
        ]);
    }
    region_from_ring(&points)
}

/// Egg region.
pub fn egg(width: Real, length: Real, segments: usize) -> CurveRegion2 {
    if segments < 3 || !positive_real(&width) || !positive_real(&length) {
        return empty();
    }
    let denominator = Real::from(segments as u64);
    let raw = (0..segments)
        .filter_map(|index| {
            let angle = Real::tau() * (Real::from(index as u64) / &denominator).ok()?;
            let sine = angle.clone().sin();
            let cosine = angle.cos();
            let correction = (cosine.clone() * cosine.clone() / Real::from(5_u8)).ok()?;
            Some([-sine, cosine + correction])
        })
        .collect::<Vec<_>>();
    if raw.len() != segments {
        return empty();
    }
    let quarter_turns = segments / 4;
    let remainder = segments % 4;
    let min_x_index = quarter_turns + usize::from(remainder >= 2);
    let max_x_index = 3 * quarter_turns + ((3 * remainder + 2) / 4);
    let min_y_index = segments / 2;
    let min_x = raw[min_x_index][0].clone();
    let max_x = raw[max_x_index][0].clone();
    let min_y = raw[min_y_index][1].clone();
    let max_y = raw[0][1].clone();
    let (Some(x_scale), Some(y_scale), Some(x_midpoint), Some(y_midpoint)) = (
        (width.clone() / (max_x.clone() - min_x.clone())).ok(),
        (length.clone() / (max_y.clone() - min_y.clone())).ok(),
        ((min_x + max_x) / Real::from(2_u8)).ok(),
        ((min_y + max_y) / Real::from(2_u8)).ok(),
    ) else {
        return empty();
    };
    let mut points = raw
        .into_iter()
        .map(|[x, y]| {
            [
                (x - x_midpoint.clone()) * x_scale.clone(),
                (y - y_midpoint.clone()) * y_scale.clone(),
            ]
        })
        .collect::<Vec<_>>();
    let (Some(half_width), Some(half_length)) = (
        (width / Real::from(2_u8)).ok(),
        (length / Real::from(2_u8)).ok(),
    ) else {
        return empty();
    };
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
    region_from_ring(&points)
}

/// Rounded rectangle.
pub fn rounded_rectangle(
    width: Real,
    height: Real,
    corner_radius: Real,
    corner_segments: usize,
) -> CurveRegion2 {
    if !positive_real(&width) || !positive_real(&height) || !nonnegative_real(&corner_radius) {
        return empty();
    }
    let half_width = (&width / Real::from(2_u8)).expect("two is nonzero");
    let half_height = (&height / Real::from(2_u8)).expect("two is nonzero");
    let radius = match real_cmp(&corner_radius, &half_width) {
        Some(Ordering::Greater) => half_width,
        Some(Ordering::Equal | Ordering::Less) => corner_radius,
        None => return empty(),
    };
    let radius = match real_cmp(&radius, &half_height) {
        Some(Ordering::Greater) => half_height,
        Some(Ordering::Equal | Ordering::Less) => radius,
        None => return empty(),
    };
    if corner_segments == 0 || !positive_real(&radius) {
        return rectangle(width, height);
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
    let Some(point_capacity) = corner_segments.checked_mul(4) else {
        return empty();
    };
    let mut points = Vec::with_capacity(point_capacity);
    let corner_count = centers.len();
    for (corner_index, (cx, cy, start)) in centers.into_iter().enumerate() {
        for index in 0..=corner_segments {
            // Adjacent analytic corner intervals share their endpoint, and the
            // last endpoint closes onto the first. Omit those duplicates by
            // construction instead of making repeated equality decisions over
            // transcendental coordinates.
            if (corner_index > 0 && index == 0)
                || (corner_index + 1 == corner_count && index == corner_segments)
            {
                continue;
            }
            let Some(fraction) =
                (Real::from(index as u64) / Real::from(corner_segments as u64)).ok()
            else {
                return empty();
            };
            let angle = start.clone() + half_pi.clone() * fraction;
            points.push([
                cx.clone() + radius.clone() * angle.clone().cos(),
                cy.clone() + radius.clone() * angle.sin(),
            ]);
        }
    }
    region_from_ring(&points)
}

/// Squircle region.
pub fn squircle(width: Real, height: Real, segments: usize) -> CurveRegion2 {
    if segments < 3 || !positive_real(&width) || !positive_real(&height) {
        return empty();
    }
    let (Some(rx), Some(ry)) = (
        (width / Real::from(2_u8)).ok(),
        (height / Real::from(2_u8)).ok(),
    ) else {
        return empty();
    };
    let signed_root = |value: Real| {
        let negative = match crate::hyper_math::hreal_sign(&value)? {
            hyperreal::RealSign::Negative => true,
            hyperreal::RealSign::Zero | hyperreal::RealSign::Positive => false,
        };
        value
            .abs()
            .sqrt()
            .ok()
            .map(|root| if negative { -root } else { root })
    };
    let denominator = Real::from(segments as u64);
    let points = (0..segments)
        .filter_map(|index| {
            let angle = Real::tau() * (Real::from(index as u64) / &denominator).ok()?;
            Some([
                rx.clone() * signed_root(angle.clone().cos())?,
                ry.clone() * signed_root(angle.sin())?,
            ])
        })
        .collect::<Vec<_>>();
    if points.len() == segments {
        region_from_ring(&points)
    } else {
        empty()
    }
}

/// Keyhole region.
pub fn keyhole(
    circle_radius: Real,
    handle_width: Real,
    handle_height: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError> {
    let decisions = GeometryDecisions::new(context);
    if segments < 3
        || !positive_real(&circle_radius)
        || !positive_real(&handle_width)
        || !positive_real(&handle_height)
    {
        return Ok(decisions.finish(empty()));
    }
    let handle_x = -(handle_width.clone() / Real::from(2_u8)).expect("two is nonzero");
    let handle = translated(
        &rectangle(handle_width, handle_height),
        handle_x,
        Real::zero(),
    );
    let region = decisions.consume_curve(
        circle(circle_radius, segments).try_union(&handle, decisions.curve_policy())?,
    );
    Ok(decisions.finish(region))
}

/// Reuleaux polygon.
pub fn reuleaux(
    sides: usize,
    diameter: Real,
    circle_segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError> {
    let decisions = GeometryDecisions::new(context);
    if sides < 3 || sides.is_multiple_of(2) || circle_segments < 6 || !positive_real(&diameter)
    {
        return Ok(decisions.finish(empty()));
    }
    let Some(double_sides) = sides.checked_mul(2) else {
        return Ok(decisions.finish(empty()));
    };
    let Some(half_angle) = (Real::pi() / Real::from(double_sides as u64)).ok() else {
        return Ok(decisions.finish(empty()));
    };
    let Some(circumradius) = (diameter.clone() / (Real::from(2_u8) * half_angle.cos())).ok()
    else {
        return Ok(decisions.finish(empty()));
    };
    let denominator = Real::from(sides as u64);
    let mut result: Option<CurveRegion2> = None;
    for index in 0..sides {
        let angle = Real::tau()
            * (Real::from(index as u64) / &denominator)
                .expect("the validated side count is nonzero");
        let disk = translated(
            &circle(diameter.clone(), circle_segments),
            circumradius.clone() * angle.clone().cos(),
            circumradius.clone() * angle.sin(),
        );
        result = Some(match result {
            None => disk,
            Some(current) => decisions
                .consume_curve(current.try_intersection(&disk, decisions.curve_policy())?),
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
) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError> {
    let decisions = GeometryDecisions::new(context);
    if segments < 3 || !positive_real(&inner_diameter) || !positive_real(&thickness) {
        return Ok(decisions.finish(empty()));
    }
    let Some(inner_radius) = (inner_diameter / Real::from(2_u8)).ok() else {
        return Ok(decisions.finish(empty()));
    };
    let inner = circle(inner_radius.clone(), segments);
    let region = decisions.consume_curve(
        circle(inner_radius + thickness, segments)
            .try_difference(&inner, decisions.curve_policy())?,
    );
    Ok(decisions.finish(region))
}

/// Circular sector.
pub fn pie_slice(
    radius: Real,
    start_angle_deg: Real,
    end_angle_deg: Real,
    segments: usize,
) -> CurveRegion2 {
    if segments == 0 || !positive_real(&radius) {
        return empty();
    }
    let sweep = end_angle_deg - start_angle_deg.clone();
    let absolute_sweep = sweep.clone().abs();
    if !positive_real(&absolute_sweep) {
        return empty();
    }
    match real_cmp(&absolute_sweep, &Real::from(360_u16)) {
        Some(Ordering::Greater) | None => return empty(),
        Some(Ordering::Equal) => return circle(radius, segments.max(3)),
        Some(Ordering::Less) => {},
    }
    let (Some(start), Some(sweep)) = (
        (start_angle_deg * Real::pi() / Real::from(180_u16)).ok(),
        (sweep * Real::pi() / Real::from(180_u16)).ok(),
    ) else {
        return empty();
    };
    let mut points = vec![[Real::zero(), Real::zero()]];
    for index in 0..=segments {
        let Ok(fraction) = Real::from(index as u64) / Real::from(segments as u64) else {
            return empty();
        };
        let angle = start.clone() + sweep.clone() * fraction;
        points.push([
            radius.clone() * angle.clone().cos(),
            radius.clone() * angle.sin(),
        ]);
    }
    region_from_ring(&points)
}

/// Heart region.
pub fn heart(width: Real, height: Real, segments: usize) -> CurveRegion2 {
    if segments < 8 || !positive_real(&width) || !positive_real(&height) {
        return empty();
    }
    let denominator = Real::from(segments as u64);
    let raw = (0..segments)
        .filter_map(|index| {
            let angle = Real::tau() * (Real::from(index as u64) / &denominator).ok()?;
            let sine = angle.clone().sin();
            let cosine = angle.cos();
            let cosine_squared = cosine.clone() * cosine.clone();
            Some([
                Real::from(16_u8) * sine.clone() * sine.clone() * sine,
                Real::from(4_u8) + Real::from(19_u8) * cosine.clone()
                    - Real::from(2_u8) * cosine_squared.clone()
                    - Real::from(8_u8) * cosine_squared.clone() * cosine
                    - Real::from(8_u8) * cosine_squared.clone() * cosine_squared,
            ])
        })
        .collect::<Vec<_>>();
    if raw.len() != segments {
        return empty();
    }
    let Some(quarter_rounding_numerator) = segments.checked_add(2) else {
        return empty();
    };
    let max_x_index = quarter_rounding_numerator / 4;
    let min_x_index = (segments - max_x_index) % segments;
    let min_x = raw[min_x_index][0].clone();
    let max_x = raw[max_x_index][0].clone();
    let mut lower = 0;
    let mut upper = segments / 2;
    while lower < upper {
        let middle = lower + (upper - lower) / 2;
        match real_cmp(&raw[middle][1], &raw[middle + 1][1]) {
            Some(Ordering::Less) => lower = middle + 1,
            Some(Ordering::Equal | Ordering::Greater) => upper = middle,
            None => return empty(),
        }
    }
    let min_y = raw[segments / 2][1].clone();
    let max_y = raw[lower][1].clone();
    let span_x = max_x - min_x.clone();
    let span_y = max_y - min_y.clone();
    let points = raw
        .into_iter()
        .filter_map(|[x, y]| {
            Some([
                width.clone() * ((x - min_x.clone()) / span_x.clone()).ok()?,
                height.clone() * ((y - min_y.clone()) / span_y.clone()).ok()?,
            ])
        })
        .collect::<Vec<_>>();
    if points.len() == segments {
        region_from_ring(&points)
    } else {
        empty()
    }
}

/// Crescent region.
pub fn crescent(
    outer_radius: Real,
    inner_radius: Real,
    offset: Real,
    segments: usize,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError> {
    let decisions = GeometryDecisions::new(context);
    if segments < 6
        || !positive_real(&inner_radius)
        || real_cmp(&outer_radius, &inner_radius) != Some(Ordering::Greater)
    {
        return Ok(decisions.finish(empty()));
    }
    let outer = circle(outer_radius, segments);
    let inner = translated(&circle(inner_radius, segments), offset, Real::zero());
    let region =
        decisions.consume_curve(outer.try_difference(&inner, decisions.curve_policy())?);
    Ok(decisions.finish(region))
}

/// Superformula-derived filled region.
#[allow(clippy::too_many_arguments)]
pub fn supershape(
    a: Real,
    b: Real,
    m: Real,
    n1: Real,
    n2: Real,
    n3: Real,
    segments: usize,
) -> CurveRegion2 {
    if segments < 3 || !positive_real(&a) || !positive_real(&b) {
        return empty();
    }
    let Some(exponent) = (Real::from(-1_i8) / n1).ok() else {
        return empty();
    };
    let denominator = Real::from(segments as u64);
    let points = (0..segments)
        .filter_map(|index| {
            let theta = Real::tau() * (Real::from(index as u64) / &denominator).ok()?;
            let angle = (m.clone() * theta.clone() / Real::from(4_u8)).ok()?;
            let first = (angle.clone().cos().abs() / a.clone())
                .ok()?
                .pow(n2.clone())
                .ok()?;
            let second = (angle.sin().abs() / b.clone()).ok()?.pow(n3.clone()).ok()?;
            let radius = (first + second).pow(exponent.clone()).ok()?;
            Some([radius.clone() * theta.clone().cos(), radius * theta.sin()])
        })
        .collect::<Vec<_>>();
    if points.len() == segments {
        region_from_ring(&points)
    } else {
        empty()
    }
}

/// Circular region with a rectangular keyway.
pub fn circle_with_keyway(
    radius: Real,
    segments: usize,
    key_width: Real,
    key_depth: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError> {
    let decisions = GeometryDecisions::new(context);
    let diameter = Real::from(2_u8) * radius.clone();
    if segments < 3
        || !positive_real(&radius)
        || !positive_real(&key_width)
        || !positive_real(&key_depth)
        || real_cmp(&key_width, &diameter) != Some(Ordering::Less)
        || real_cmp(&key_depth, &diameter) != Some(Ordering::Less)
    {
        return Ok(decisions.finish(empty()));
    }
    let key_y = -(key_width.clone() / Real::from(2_u8)).expect("two is nonzero");
    let cutter = translated(
        &rectangle(key_depth.clone(), key_width),
        radius.clone() - key_depth,
        key_y,
    );
    let region = decisions.consume_curve(
        circle(radius, segments).try_difference(&cutter, decisions.curve_policy())?,
    );
    Ok(decisions.finish(region))
}

/// Circular region with one flat chord.
pub fn circle_with_flat(
    radius: Real,
    segments: usize,
    flat_distance: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError> {
    let decisions = GeometryDecisions::new(context);
    if segments < 3
        || !positive_real(&radius)
        || !nonnegative_real(&flat_distance)
        || real_cmp(&flat_distance, &radius) != Some(Ordering::Less)
    {
        return Ok(decisions.finish(empty()));
    }
    let diameter = Real::from(2_u8) * radius.clone();
    let clip = translated(
        &rectangle(diameter, radius.clone() + flat_distance.clone()),
        -radius.clone(),
        -flat_distance,
    );
    let region = decisions.consume_curve(
        circle(radius, segments).try_intersection(&clip, decisions.curve_policy())?,
    );
    Ok(decisions.finish(region))
}

/// Circular region with two opposing flat chords.
pub fn circle_with_two_flats(
    radius: Real,
    segments: usize,
    flat_distance: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError> {
    let decisions = GeometryDecisions::new(context);
    if segments < 3
        || !positive_real(&radius)
        || !nonnegative_real(&flat_distance)
        || real_cmp(&flat_distance, &radius) != Some(Ordering::Less)
    {
        return Ok(decisions.finish(empty()));
    }
    let clip = translated(
        &rectangle(
            Real::from(2_u8) * radius.clone(),
            Real::from(2_u8) * flat_distance.clone(),
        ),
        -radius.clone(),
        -flat_distance,
    );
    let region = decisions.consume_curve(
        circle(radius, segments).try_intersection(&clip, decisions.curve_policy())?,
    );
    Ok(decisions.finish(region))
}

/// Involute gear region.
pub fn involute_gear(
    module: Real,
    teeth: usize,
    pressure_angle_degrees: Real,
    clearance: Real,
    backlash: Real,
    segments_per_flank: usize,
) -> CurveRegion2 {
    if teeth < 4
        || segments_per_flank < 2
        || !positive_real(&module)
        || !nonnegative_real(&clearance)
        || !nonnegative_real(&backlash)
        || !positive_real(&pressure_angle_degrees)
        || real_cmp(&pressure_angle_degrees, &Real::from(90_u8)) != Some(Ordering::Less)
    {
        return empty();
    }
    sampled_involute_gear(
        &module,
        teeth,
        &pressure_angle_degrees,
        &clearance,
        &backlash,
        segments_per_flank,
    )
}

/// Cycloidal gear region.
///
/// Returns an empty region when the requested first-lobe flanks would meet or
/// cross before reaching the addendum circle or the neighboring root gap.
pub fn cycloidal_gear(
    module: Real,
    teeth: usize,
    generating_radius: Real,
    clearance: Real,
    segments_per_flank: usize,
) -> CurveRegion2 {
    if teeth < 3
        || !positive_real(&generating_radius)
        || segments_per_flank < 2
        || !positive_real(&module)
        || !nonnegative_real(&clearance)
    {
        return empty();
    }
    sampled_cycloidal_gear(
        &module,
        teeth,
        &generating_radius,
        &clearance,
        segments_per_flank,
    )
}

/// Linear involute rack region.
pub fn involute_rack(
    module: Real,
    teeth: usize,
    pressure_angle_degrees: Real,
    clearance: Real,
    backlash: Real,
) -> CurveRegion2 {
    if teeth == 0
        || !positive_real(&module)
        || !positive_real(&pressure_angle_degrees)
        || real_cmp(&pressure_angle_degrees, &Real::from(90_u8)) != Some(Ordering::Less)
        || !nonnegative_real(&clearance)
        || !nonnegative_real(&backlash)
    {
        return empty();
    }
    let Some(point_capacity) = teeth.checked_mul(6).and_then(|count| count.checked_add(2))
    else {
        return empty();
    };
    let pressure_angle = (pressure_angle_degrees * Real::pi() / Real::from(180_u16))
        .expect("degrees denominator is nonzero");
    let pitch = Real::pi() * module.clone();
    let dedendum = module.clone()
        * (Real::from(5_u8) / Real::from(4_u8)).expect("four is nonzero")
        + clearance;
    let root = -dedendum.clone();
    let Ok(half_pitch) = pitch.clone() / Real::from(2_u8) else {
        return empty();
    };
    let tooth_thickness = half_pitch - backlash;
    let half_thickness = (tooth_thickness.clone() / Real::from(2_u8)).expect("two is nonzero");
    let Some(tangent) = (pressure_angle.clone().sin() / pressure_angle.cos()).ok() else {
        return empty();
    };
    let root_slant = dedendum * tangent.clone();
    let tip_slant = module.clone() * tangent;
    let tip_width = tooth_thickness.clone() - Real::from(2_u8) * tip_slant.clone();
    let root_space =
        pitch.clone() - tooth_thickness.clone() - Real::from(2_u8) * root_slant.clone();
    if !positive_real(&tooth_thickness)
        || !positive_real(&tip_width)
        || !positive_real(&root_space)
    {
        return empty();
    }
    let first_x = -half_thickness.clone() - root_slant.clone();
    let mut points = Vec::with_capacity(point_capacity);
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
    region_from_ring(&points)
}

/// Linear cycloidal rack region.
pub fn cycloidal_rack(
    module: Real,
    teeth: usize,
    clearance: Real,
    segments_per_flank: usize,
) -> CurveRegion2 {
    if teeth == 0
        || segments_per_flank < 4
        || !positive_real(&module)
        || !nonnegative_real(&clearance)
    {
        return empty();
    }
    let half = (Real::one() / Real::from(2_u8)).expect("two is nonzero");
    let generating_radius = module.clone() * half.clone();
    let pitch = Real::pi() * module.clone();
    let root = -(module.clone()
        * (Real::from(5_u8) / Real::from(4_u8)).expect("four is nonzero")
        + clearance);
    let left = -half.clone() * pitch.clone();
    let right = (Real::from(teeth as u64) - half.clone()) * pitch.clone();
    let Some(lobe_capacity) = segments_per_flank.checked_add(1) else {
        return empty();
    };
    let mut lobe = Vec::with_capacity(lobe_capacity);
    for sample in 0..=segments_per_flank {
        let Some(fraction) = exact_ratio(sample, segments_per_flank) else {
            return empty();
        };
        let theta = Real::tau() * fraction;
        lobe.push([
            generating_radius.clone() * (theta.clone() - theta.clone().sin()),
            generating_radius.clone() * (Real::one() - theta.cos()),
        ]);
    }
    let Some(top_capacity) = teeth
        .checked_mul(segments_per_flank)
        .and_then(|count| count.checked_add(1))
    else {
        return empty();
    };
    let mut top = Vec::with_capacity(top_capacity);
    for tooth in 0..teeth {
        let tooth_left = (Real::from(tooth as u64) - half.clone()) * pitch.clone();
        for (sample, point) in lobe.iter().enumerate() {
            if tooth > 0 && sample == 0 {
                continue;
            }
            top.push([tooth_left.clone() + point[0].clone(), point[1].clone()]);
        }
    }
    let Some(point_capacity) = top.len().checked_add(4) else {
        return empty();
    };
    let mut points = Vec::with_capacity(point_capacity);
    let backing = root.clone() - module;
    points.push([left.clone(), backing.clone()]);
    points.push([right.clone(), backing]);
    points.push([right, root.clone()]);
    points.extend(top.into_iter().rev());
    points.push([left, root]);
    region_from_ring(&points)
}

/// NACA four-digit airfoil region.
pub fn airfoil_naca4(
    max_camber: Real,
    camber_position: Real,
    thickness: Real,
    chord: Real,
    samples: usize,
) -> CurveRegion2 {
    if samples < 10
        || !nonnegative_real(&max_camber)
        || real_cmp(&max_camber, &Real::from(10_u8)) != Some(Ordering::Less)
        || !nonnegative_real(&camber_position)
        || real_cmp(&camber_position, &Real::from(10_u8)) != Some(Ordering::Less)
        || !positive_real(&thickness)
        || real_cmp(&thickness, &Real::from(100_u8)) != Some(Ordering::Less)
        || !positive_real(&chord)
    {
        return empty();
    }
    let (Some(m), Some(p), Some(t)) = (
        (max_camber / Real::from(100_u8)).ok(),
        (camber_position / Real::from(10_u8)).ok(),
        (thickness / Real::from(100_u8)).ok(),
    ) else {
        return empty();
    };
    let cambered = positive_real(&m);
    if cambered && (!positive_real(&p) || real_cmp(&p, &Real::one()) != Some(Ordering::Less)) {
        return empty();
    }
    let coefficient = |value: f64| Real::try_from(value).expect("finite NACA coefficient");
    let Some(sample_capacity) = samples.checked_add(1) else {
        return empty();
    };
    let mut upper = Vec::with_capacity(sample_capacity);
    let mut lower = Vec::with_capacity(sample_capacity);
    for index in 0..=samples {
        let Some(x) = (Real::from(index as u64) / Real::from(samples as u64)).ok() else {
            return empty();
        };
        let x2 = x.clone() * x.clone();
        let x3 = x2.clone() * x.clone();
        let x4 = x3.clone() * x.clone();
        let Some(root_x) = x.clone().sqrt().ok() else {
            return empty();
        };
        let yt = Real::from(5_u8)
            * t.clone()
            * (coefficient(0.2969) * root_x
                - coefficient(0.1260) * x.clone()
                - coefficient(0.3516) * x2.clone()
                + coefficient(0.2843) * x3
                - coefficient(0.1015) * x4);
        let (yc, dy) = if cambered {
            let Some(x_ordering) = real_cmp(&x, &p) else {
                return empty();
            };
            if x_ordering == Ordering::Less {
                let p2 = p.clone() * p.clone();
                let (Some(y_factor), Some(slope_factor)) = (
                    (m.clone() / p2.clone()).ok(),
                    (Real::from(2_u8) * m.clone() / p2).ok(),
                ) else {
                    return empty();
                };
                (
                    y_factor * (Real::from(2_u8) * p.clone() * x.clone() - x2.clone()),
                    slope_factor * (p.clone() - x.clone()),
                )
            } else {
                let q = Real::one() - p.clone();
                let denominator = q.clone() * q;
                let (Some(y_factor), Some(slope_factor)) = (
                    (m.clone() / denominator.clone()).ok(),
                    (Real::from(2_u8) * m.clone() / denominator).ok(),
                ) else {
                    return empty();
                };
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
        let Some(theta) = dy.atan().ok() else {
            return empty();
        };
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
    region_from_ring(&upper)
}

fn sampled_involute_gear(
    module: &Real,
    teeth: usize,
    pressure_angle_degrees: &Real,
    clearance: &Real,
    backlash: &Real,
    segments_per_flank: usize,
) -> CurveRegion2 {
    let points = (|| -> Option<Vec<[Real; 2]>> {
        let two = Real::from(2_u8);
        let four = Real::from(4_u8);
        let tooth_count = Real::from(teeth as u64);
        let pressure_angle =
            (pressure_angle_degrees.clone() * Real::pi() / Real::from(180_u16)).ok()?;
        let pitch_radius = (module.clone() * tooth_count.clone() / two.clone()).ok()?;
        let base_radius = pitch_radius.clone() * pressure_angle.cos();
        let outer_radius = pitch_radius.clone() + module.clone();
        let root_radius = pitch_radius.clone()
            - (module.clone() * Real::from(5_u8) / four.clone()).ok()?
            - clearance.clone();
        let angular_pitch = (Real::tau() / tooth_count).ok()?;
        let backlash_angle = (backlash.clone() / pitch_radius.clone()).ok()?;
        let half_tooth_angle =
            (((angular_pitch.clone() / two.clone()).ok()? - backlash_angle) / two).ok()?;
        if !positive_real(&base_radius)
            || !positive_real(&root_radius)
            || !positive_real(&half_tooth_angle)
        {
            return None;
        }
        let involute_angle = |radius: &Real| -> Option<Real> {
            let ratio = (radius.clone() / base_radius.clone()).ok()?;
            let parameter = (ratio.clone() * ratio - Real::one()).sqrt().ok()?;
            Some(parameter.clone() - parameter.atan().ok()?)
        };
        let (flank_start_radius, has_root_transition) =
            match real_cmp(&root_radius, &base_radius)? {
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
        if real_cmp(&right_start, &left_start)? != Ordering::Less
            || real_cmp(&right_tip, &left_tip)? != Ordering::Less
        {
            return None;
        }

        let mut flank_samples = Vec::with_capacity(segments_per_flank);
        for sample in 1..segments_per_flank {
            let fraction = exact_ratio(sample, segments_per_flank)?;
            let radius = flank_start_radius.clone()
                + fraction * (outer_radius.clone() - flank_start_radius.clone());
            flank_samples.push((radius.clone(), involute_angle(&radius)?));
        }
        flank_samples.push((outer_radius.clone(), outer_involute));

        let mut tooth = Vec::with_capacity(segments_per_flank.checked_mul(4)?.checked_add(3)?);
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
        repeat_exact_rotational_pattern(&tooth, teeth)
    })();
    points.map_or_else(empty, |points| region_from_ring(&points))
}

fn sampled_cycloidal_gear(
    module: &Real,
    teeth: usize,
    generating_radius: &Real,
    clearance: &Real,
    segments_per_flank: usize,
) -> CurveRegion2 {
    let points = (|| -> Option<Vec<[Real; 2]>> {
        let two = Real::from(2_u8);
        let pitch_radius = (module.clone() * Real::from(teeth as u64) / two.clone()).ok()?;
        let outer_radius = pitch_radius.clone() + module.clone();
        let root_radius = pitch_radius.clone()
            - (module.clone() * Real::from(5_u8) / Real::from(4_u8)).ok()?
            - clearance.clone();
        let twice_generator = two.clone() * generating_radius.clone();
        if !positive_real(&root_radius)
            || real_cmp(&twice_generator, &pitch_radius)? != Ordering::Less
        {
            return None;
        }
        let epicycle_radius = pitch_radius.clone() + generating_radius.clone();
        let hypocycle_radius = pitch_radius.clone() - generating_radius.clone();
        if real_cmp(
            &outer_radius,
            &(pitch_radius.clone() + twice_generator.clone()),
        )? == Ordering::Greater
        {
            return None;
        }
        let generated_root_radius = pitch_radius.clone() - twice_generator;
        let flank_root_radius = match real_cmp(&root_radius, &generated_root_radius)? {
            Ordering::Less => generated_root_radius,
            Ordering::Equal | Ordering::Greater => root_radius.clone(),
        };
        let square = |value: &Real| value.clone() * value.clone();
        let tip_phase = ((square(&epicycle_radius) + square(generating_radius)
            - square(&outer_radius))
            / (two.clone() * generating_radius.clone() * epicycle_radius.clone()))
        .ok()?
        .acos()
        .ok()?;
        let tip_parameter =
            (tip_phase.clone() * generating_radius.clone() / pitch_radius.clone()).ok()?;
        let root_phase = ((square(&flank_root_radius)
            - square(&hypocycle_radius)
            - square(generating_radius))
            / (two.clone() * generating_radius.clone() * hypocycle_radius.clone()))
        .ok()?
        .acos()
        .ok()?;
        let root_parameter =
            (root_phase.clone() * generating_radius.clone() / pitch_radius.clone()).ok()?;
        let epicycle_ratio = (epicycle_radius.clone() / generating_radius.clone()).ok()?;
        let hypocycle_ratio = (hypocycle_radius.clone() / generating_radius.clone()).ok()?;
        let angular_pitch = (Real::tau() / Real::from(teeth as u64)).ok()?;
        let pitch_half_thickness = (angular_pitch.clone() / Real::from(4_u8)).ok()?;
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
        let sample_capacity = segments_per_flank.checked_add(1)?;
        let mut epi = Vec::with_capacity(sample_capacity);
        let mut hypo = Vec::with_capacity(sample_capacity);
        for sample in 0..=segments_per_flank {
            let fraction = exact_ratio(sample, segments_per_flank)?;
            epi.push(epicycloid(tip_parameter.clone() * fraction.clone()));
            hypo.push(hypocycloid(-(root_parameter.clone() * fraction)));
        }

        let limit_sine = pitch_half_thickness.clone().sin();
        let limit_cosine = pitch_half_thickness.clone().cos();
        let precedes_tooth_bisector = |[x, y]: &[Real; 2]| {
            real_cmp(
                &(y.clone() * limit_cosine.clone() - x.clone() * limit_sine.clone()),
                &Real::zero(),
            ) == Some(Ordering::Less)
        };
        let negative_root_endpoint = hypo.last()?;
        let positive_root_endpoint = [
            negative_root_endpoint[0].clone(),
            -negative_root_endpoint[1].clone(),
        ];
        if !precedes_tooth_bisector(epi.last()?)
            || !precedes_tooth_bisector(&positive_root_endpoint)
        {
            return None;
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
        let right_epi = epi.iter().map(rotate).collect::<Vec<_>>();
        let right_hypo = hypo.iter().map(rotate).collect::<Vec<_>>();
        let right_tip_point = right_epi.last()?;
        let right_root_point = right_hypo.last()?;
        let right_tip = right_tip_point[1].clone().atan2(right_tip_point[0].clone());
        let right_root = right_root_point[1].clone().atan2(right_root_point[0].clone());
        let left_tip = -right_tip.clone();
        let left_root = -right_root.clone();
        let next_right_root = angular_pitch.clone() + right_root.clone();
        let mut tooth = Vec::with_capacity(segments_per_flank.checked_mul(6)?.checked_add(2)?);
        let has_root_transition =
            real_cmp(&flank_root_radius, &root_radius)? == Ordering::Greater;
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
        repeat_exact_rotational_pattern(&tooth, teeth)
    })();
    points.map_or_else(empty, |points| region_from_ring(&points))
}

/// Exact arbitrary-degree Bezier path.
///
/// Closed controls should be passed to [`bezier_region`] instead.
pub fn bezier_path(control: &[[Real; 2]], display_segments: usize) -> Option<CurvePath2> {
    if control.len() < 2 || display_segments < 1 {
        return None;
    }
    let points = control
        .iter()
        .map(|point| Point2::new(point[0].clone(), point[1].clone()))
        .collect::<Vec<_>>();
    let curve = match points.as_slice() {
        [start, end] => LineSeg2::try_new(start.clone(), end.clone())
            .ok()
            .map(Curve2::from),
        [start, control, end] => Some(Curve2::from(QuadraticBezier2::new(
            start.clone(),
            control.clone(),
            end.clone(),
        ))),
        [start, control1, control2, end] => Some(Curve2::from(CubicBezier2::new(
            start.clone(),
            control1.clone(),
            control2.clone(),
            end.clone(),
        ))),
        _ => RationalBezier2::try_new(points, vec![Real::one(); control.len()])
            .ok()
            .map(Curve2::from),
    }?;
    CurvePath2::try_new(vec![curve]).ok()
}

/// Closed arbitrary-degree Bezier boundary as a filled region.
pub fn bezier_region(control: &[[Real; 2]], display_segments: usize) -> CurveRegion2 {
    let Some(path) = bezier_path(control, display_segments) else {
        return empty();
    };
    if points_equal(path.start(), path.end()) != Some(true) {
        return empty();
    }
    CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path))
        .unwrap_or_else(|_| empty())
}

/// Exact open-uniform polynomial B-spline path.
pub fn bspline_path(
    control: &[[Real; 2]],
    degree: usize,
    display_segments_per_span: usize,
) -> Option<CurvePath2> {
    let minimum_control_count = degree.checked_add(1)?;
    if control.len() < minimum_control_count || display_segments_per_span < 1 {
        return None;
    }
    let n = control.len() - 1;
    let m = n.checked_add(degree)?.checked_add(1)?;
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
    let spline = PolynomialSplineCurve2::try_new(degree, points, knots).ok()?;
    CurvePath2::try_new(vec![Curve2::from(spline)]).ok()
}

/// Hilbert infill returned as native open curve strings.
pub fn hilbert_strings(
    boundary: &CurveRegion2,
    order: usize,
    padding: Real,
) -> Vec<CurveString2> {
    if order == 0
        || order > 10
        || !matches!(
            crate::hyper_math::hreal_sign(&padding),
            Some(hyperreal::RealSign::Positive | hyperreal::RealSign::Zero)
        )
    {
        return Vec::new();
    }
    let Ok(bounds) = try_bounding_box(boundary) else {
        return Vec::new();
    };
    let width =
        bounds.maxs.x.clone() - bounds.mins.x.clone() - Real::from(2_u8) * padding.clone();
    let height =
        bounds.maxs.y.clone() - bounds.mins.y.clone() - Real::from(2_u8) * padding.clone();
    if !matches!(
        crate::hyper_math::hreal_sign(&width),
        Some(hyperreal::RealSign::Positive)
    ) || !matches!(
        crate::hyper_math::hreal_sign(&height),
        Some(hyperreal::RealSign::Positive)
    ) {
        return Vec::new();
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
    let points = (0..size * size)
        .filter_map(|distance| {
            let (x, y) = point(size, distance);
            // Hilbert visits cell centers, not lower-left cell corners. This
            // keeps the path symmetrically inset from all four usable bounds.
            let u = ((Real::from(x) + half.clone()) / &denominator).ok()?;
            let v = ((Real::from(y) + half.clone()) / &denominator).ok()?;
            Some((
                origin_x.clone() + width.clone() * u,
                origin_y.clone() + height.clone() * v,
            ))
        })
        .collect::<Vec<_>>();
    if points.len() != (size * size) as usize {
        return Vec::new();
    }
    let mut runs = Vec::<Vec<(Real, Real)>>::new();
    let mut run = Vec::new();
    for pair in points.windows(2) {
        let midpoint_x = ((&pair[0].0 + &pair[1].0) / Real::from(2_u8)).ok();
        let midpoint_y = ((&pair[0].1 + &pair[1].1) / Real::from(2_u8)).ok();
        let Some(keep) = midpoint_x
            .zip(midpoint_y)
            .and_then(|(x, y)| contains_xy(boundary, x, y))
        else {
            return Vec::new();
        };
        if keep {
            if run.is_empty() {
                run.push(pair[0].clone());
            }
            run.push(pair[1].clone());
        } else if run.len() >= 2 {
            runs.push(std::mem::take(&mut run));
        }
    }
    if run.len() >= 2 {
        runs.push(run);
    }
    let strings = runs
        .into_iter()
        .map(|run| {
            CurveString2::from_real_point_iter(run.into_iter().map(|(x, y)| [x, y])).ok()
        })
        .collect::<Option<Vec<_>>>();
    strings.unwrap_or_default()
}

/// Sharp regularized offset of a native filled region.
#[cfg(feature = "offset")]
pub fn offset(
    input: &CurveRegion2,
    distance: Real,
    policy: &CurvePolicy,
) -> Result<CurveOutcome<CurveRegion2>, crate::errors::CurveOffsetError> {
    input.offset(distance, policy).map_err(Into::into)
}

/// Rounded regularized offset of a native filled region.
#[cfg(feature = "offset")]
pub fn offset_rounded(
    input: &CurveRegion2,
    distance: Real,
    policy: &CurvePolicy,
) -> Result<CurveOutcome<CurveRegion2>, crate::errors::CurveOffsetError> {
    offset(input, distance, policy)
}

/// Triangulates a filled region as a flat native triangle surface.
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
    for profile in try_finite_profiles_with(input, decisions)? {
        let faces = decisions.consume_curve(
            profile
                .triangulate(decisions.curve_policy())
                .map_err(|error| ValidationError::Geometry(error.to_string()))?,
        );
        for face in faces {
            let Some(points) = face
                .into_iter()
                .map(|[x, y]| {
                    Some(Point3::new(
                        Real::try_from(x).ok()?,
                        Real::try_from(y).ok()?,
                        Real::zero(),
                    ))
                })
                .collect::<Option<Vec<_>>>()
            else {
                return Err(ValidationError::InvalidArguments);
            };
            // Hypertri certifies the exact finite-profile triangulation.
            if !emit_certified_triangle(
                &mut positions,
                &mut triangles,
                points.try_into().expect("triangulation emits triangles"),
                false,
            ) {
                return Err(ValidationError::InvalidArguments);
            }
        }
    }
    validate_surface_mesh(TriangleMesh::new(positions, triangles), decisions)
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
    decisions.classify_curve("finite curve projection", |policy| {
        input.project_to_finite_profiles_exact(&options, policy)
    })
}

/// Linear extrusion that reports invalid or uncertain geometry.
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
        return Err(ValidationError::Geometry(
            "extrusion direction has zero Z component".into(),
        ));
    }
    let flip = direction_sign == hyperlimit::Sign::Negative;
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let point = |[x, y]: [f64; 2]| -> Option<Point3> {
        Some(Point3::new(
            Real::try_from(x).ok()?,
            Real::try_from(y).ok()?,
            Real::zero(),
        ))
    };

    for profile in try_finite_profiles_with(region, decisions)? {
        let cap_faces =
            decisions.consume_curve(profile.triangulate(decisions.curve_policy()).map_err(
                |error| ValidationError::Geometry(format!("extrusion cap: {error}")),
            )?);
        for face in cap_faces {
            let [Some(a), Some(b), Some(c)] = face.map(point) else {
                return Err(ValidationError::Geometry(
                    "extrusion cap contains a non-finite projected coordinate".into(),
                ));
            };
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
        for (ring, is_hole) in std::iter::once((profile.material(), false))
            .chain(profile.holes().iter().map(|ring| (ring, true)))
        {
            // Hypertri emits caps with a normalized positive outer winding and
            // negative hole winding. Exact curve-region projection preserves
            // authored loop orientation, so align each side ring with the cap
            // role instead of assuming every material loop arrived CCW.
            let reverse_ring = match finite_ring_sign(ring, decisions)? {
                hyperlimit::Sign::Positive => is_hole,
                hyperlimit::Sign::Negative => !is_hole,
                hyperlimit::Sign::Zero => {
                    return Err(ValidationError::Geometry(
                        "extrusion side ring has zero signed area".into(),
                    ));
                },
            };
            for edge in ring.points().windows(2) {
                let (Some(mut bottom_a), Some(mut bottom_b)) =
                    (point(edge[0]), point(edge[1]))
                else {
                    return Err(ValidationError::Geometry(
                        "extrusion side contains a non-finite projected coordinate".into(),
                    ));
                };
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
            let cap_faces = decisions.consume_curve(
                profile
                    .triangulate(decisions.curve_policy())
                    .map_err(|error| {
                        ValidationError::Geometry(format!("revolution cap: {error}"))
                    })?,
            );
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
            let cap_faces = decisions.consume_curve(
                profile
                    .triangulate(decisions.curve_policy())
                    .map_err(|error| {
                        ValidationError::Geometry(format!("sweep cap: {error}"))
                    })?,
            );
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
        let cap_faces =
            decisions.consume_curve(profile.triangulate(decisions.curve_policy()).map_err(
                |error| ValidationError::Geometry(format!("twisted-extrusion cap: {error}")),
            )?);
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

/// Filled contour generated from a two-dimensional metaball field.
#[cfg(feature = "metaballs")]
pub fn metaballs(
    balls: &[(Point2, Real)],
    resolution: (usize, usize),
    iso_value: Real,
    padding: Real,
) -> CurveRegion2 {
    super::metaballs::metaballs(balls, resolution, iso_value, padding)
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

/// Hershey stroke geometry as native open curve strings.
#[cfg(feature = "hershey-text")]
pub fn hershey_strings(
    text: &str,
    font: &hypercurve::hershey::Font<'_>,
    size: Real,
) -> Vec<CurveString2> {
    hypercurve::hershey::strings(text, font, size)
}

/// Convenience CSG and construction operations on native filled regions.
pub trait CurveRegionExt: Sized {
    /// Exact regularized union.
    fn try_union(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Result<CurveOutcome<Self>, CurveBooleanError>;
    /// Exact regularized difference.
    fn try_difference(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Result<CurveOutcome<Self>, CurveBooleanError>;
    /// Exact regularized intersection.
    fn try_intersection(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Result<CurveOutcome<Self>, CurveBooleanError>;
    /// Exact regularized symmetric difference.
    fn try_xor(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Result<CurveOutcome<Self>, CurveBooleanError>;
    /// Exact planar affine transform.
    fn transformed_affine(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
        tx: &Real,
        ty: &Real,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Self>;
}

impl CurveRegionExt for CurveRegion2 {
    fn try_union(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Result<CurveOutcome<Self>, CurveBooleanError> {
        self.boolean_region(other, BooleanOp::Union, policy)
            .map_err(CurveBooleanError::from)
    }

    fn try_difference(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Result<CurveOutcome<Self>, CurveBooleanError> {
        self.boolean_region(other, BooleanOp::Difference, policy)
            .map_err(CurveBooleanError::from)
    }

    fn try_intersection(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Result<CurveOutcome<Self>, CurveBooleanError> {
        self.boolean_region(other, BooleanOp::Intersection, policy)
            .map_err(CurveBooleanError::from)
    }

    fn try_xor(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Result<CurveOutcome<Self>, CurveBooleanError> {
        self.boolean_region(other, BooleanOp::Xor, policy)
            .map_err(CurveBooleanError::from)
    }

    fn transformed_affine(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
        tx: &Real,
        ty: &Real,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Self> {
        self.transform_affine(m00, m01, m10, m11, tx, ty, policy)
    }
}

/// Applies a 3D homogeneous transform through the established curve lifting
/// boundary and returns only native filled topology.
pub fn try_transformed(
    input: &CurveRegion2,
    matrix: &Matrix4,
) -> ExactCurveResult<CurveRegion2> {
    input.transform_affine(
        &matrix.0[0][0],
        &matrix.0[0][1],
        &matrix.0[1][0],
        &matrix.0[1][1],
        &matrix.0[0][3],
        &matrix.0[1][3],
        &CurvePolicy::STRICT,
    )
}

pub fn transformed(input: &CurveRegion2, matrix: &Matrix4) -> CurveRegion2 {
    try_transformed(input, matrix).unwrap_or_else(|_| empty())
}

/// Applies an exact planar translation to a filled native region.
pub fn try_translated(
    input: &CurveRegion2,
    x: Real,
    y: Real,
) -> ExactCurveResult<CurveRegion2> {
    input.transform_affine(
        &Real::one(),
        &Real::zero(),
        &Real::zero(),
        &Real::one(),
        &x,
        &y,
        &CurvePolicy::STRICT,
    )
}

pub fn translated(input: &CurveRegion2, x: Real, y: Real) -> CurveRegion2 {
    try_translated(input, x, y).unwrap_or_else(|_| empty())
}

/// Rotates a filled native region counterclockwise about the origin in degrees.
pub fn try_rotated(input: &CurveRegion2, z_degrees: Real) -> ExactCurveResult<CurveRegion2> {
    let radians = (z_degrees * Real::pi() / Real::from(180_u16)).expect("180 is nonzero");
    let cosine = radians.clone().cos();
    let sine = radians.sin();
    input.transform_affine(
        &cosine,
        &(-sine.clone()),
        &sine,
        &cosine,
        &Real::zero(),
        &Real::zero(),
        &CurvePolicy::STRICT,
    )
}

pub fn rotated(input: &CurveRegion2, z_degrees: Real) -> CurveRegion2 {
    try_rotated(input, z_degrees).unwrap_or_else(|_| empty())
}

/// Scales a filled native region independently along the planar axes.
pub fn try_scaled(input: &CurveRegion2, x: Real, y: Real) -> ExactCurveResult<CurveRegion2> {
    input.transform_affine(
        &x,
        &Real::zero(),
        &Real::zero(),
        &y,
        &Real::zero(),
        &Real::zero(),
        &CurvePolicy::STRICT,
    )
}

pub fn scaled(input: &CurveRegion2, x: Real, y: Real) -> CurveRegion2 {
    try_scaled(input, x, y).unwrap_or_else(|_| empty())
}

/// Classifies an exact XY point, returning `None` on boundaries or uncertainty.
pub fn contains_xy(input: &CurveRegion2, x: Real, y: Real) -> Option<bool> {
    if input.is_empty() {
        return None;
    }
    match input
        .classify_point(&Point2::new(x, y), &CurvePolicy::STRICT)
        .ok()?
    {
        Classification::Decided(RegionPointLocation::Inside) => Some(true),
        Classification::Decided(RegionPointLocation::Outside) => Some(false),
        Classification::Decided(RegionPointLocation::Boundary)
        | Classification::Uncertain(_) => None,
    }
}

/// Returns the certified exact planar bounds, embedded in the XY plane.
pub fn try_bounding_box(input: &CurveRegion2) -> Result<Aabb, ValidationError> {
    if input.is_empty() {
        return Ok(Aabb::origin());
    }
    match input.bounds(&CurvePolicy::STRICT) {
        Ok(Classification::Decided(bounds)) => Ok(Aabb::new(
            Point3::new(bounds.min_x().clone(), bounds.min_y().clone(), Real::zero()),
            Point3::new(bounds.max_x().clone(), bounds.max_y().clone(), Real::zero()),
        )),
        Ok(Classification::Uncertain(reason)) => Err(ValidationError::Geometry(format!(
            "curve bounds are uncertain: {reason:?}"
        ))),
        Err(error) => Err(ValidationError::Geometry(error.to_string())),
    }
}

/// Returns exact bounds, using the origin box only for empty or invalid input.
pub fn bounding_box(input: &CurveRegion2) -> Aabb {
    try_bounding_box(input).unwrap_or_else(|_| Aabb::origin())
}

/// Returns the same native solid translation helper used by the 3D grammar.
pub fn translated_solid(mesh: &TriangleMesh, x: Real, y: Real, z: Real) -> TriangleMesh {
    solid::SolidExt::translated(mesh, x, y, z)
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn filled_constructor_and_extrusion_remain_native() {
        let region: CurveRegion2 = circle(Real::from(2_u8), 32);
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
        let square = square(Real::from(2_u8));
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

        let radial = region_from_ring(&[
            [Real::one(), -Real::one()],
            [Real::from(2_u8), -Real::one()],
            [Real::from(2_u8), Real::one()],
            [Real::one(), Real::one()],
        ]);
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
        let square = square(Real::from(2_u8));
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
        let square = square(Real::from(2_u8));
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
        let square = square(Real::from(2_u8));
        let symbolic_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
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
        let left = square(Real::from(2_u8));
        let right = rectangle(Real::from(1_u8), Real::from(3_u8));
        let union: CurveRegion2 = left
            .try_union(&right, &CurvePolicy::STRICT)
            .expect("region union")
            .into_value();
        assert!(!union.is_empty());
    }

    #[test]
    fn planar_constructors_reject_invalid_dimensions_and_ranges() {
        let negative = -Real::one();
        let empty_compound =
            |result: Result<GeometryOutcome<CurveRegion2>, CurveBooleanError>| {
                result
                    .expect("invalid compound input remains empty")
                    .value
                    .is_empty()
            };
        assert!(circle(negative.clone(), 16).is_empty());
        assert!(right_triangle(negative.clone(), Real::one()).is_empty());
        assert!(ellipse(Real::one(), negative.clone(), 16).is_empty());
        assert!(regular_ngon(5, negative.clone()).is_empty());
        assert!(
            trapezoid(Real::one(), negative.clone(), Real::one(), Real::zero()).is_empty()
        );
        assert!(star(5, Real::one(), Real::from(2_u8)).is_empty());
        assert!(empty_compound(keyhole(
            Real::one(),
            Real::zero(),
            Real::one(),
            16,
            &GeometryContext::STRICT,
        )));
        assert!(empty_compound(ring(
            Real::one(),
            negative.clone(),
            16,
            &GeometryContext::STRICT,
        )));
        assert!(empty_compound(crescent(
            Real::one(),
            Real::from(2_u8),
            Real::zero(),
            16,
            &GeometryContext::STRICT,
        )));
        assert!(empty_compound(circle_with_keyway(
            Real::one(),
            16,
            Real::from(2_u8),
            Real::one(),
            &GeometryContext::STRICT,
        )));
        assert!(empty_compound(circle_with_flat(
            Real::one(),
            16,
            Real::one(),
            &GeometryContext::STRICT,
        )));
        assert!(empty_compound(circle_with_two_flats(
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
            .is_empty()
        );
        assert!(
            airfoil_naca4(
                Real::from(2_u8),
                Real::from(10_u8),
                Real::from(12_u8),
                Real::one(),
                20,
            )
            .is_empty()
        );
        assert!(
            airfoil_naca4(
                Real::zero(),
                Real::zero(),
                Real::from(100_u8),
                Real::one(),
                20,
            )
            .is_empty()
        );
        assert!(star(usize::MAX, Real::from(2_u8), Real::one()).is_empty());
        assert!(empty_compound(reuleaux(
            usize::MAX,
            Real::one(),
            16,
            &GeometryContext::STRICT,
        )));
        assert!(teardrop(Real::one(), Real::from(2_u8), usize::MAX).is_empty());
        assert!(
            involute_rack(
                Real::one(),
                usize::MAX,
                Real::from(20_u8),
                Real::zero(),
                Real::zero(),
            )
            .is_empty()
        );
        assert!(cycloidal_rack(Real::one(), 1, Real::zero(), usize::MAX).is_empty());
        assert!(
            cycloidal_gear(
                Real::one(),
                12,
                (Real::from(3_u8) / Real::from(4_u8)).unwrap(),
                Real::zero(),
                usize::MAX,
            )
            .is_empty()
        );
        assert!(
            airfoil_naca4(
                Real::zero(),
                Real::zero(),
                Real::from(12_u8),
                Real::one(),
                usize::MAX,
            )
            .is_empty()
        );
        assert!(bspline_path(&[], usize::MAX, 1).is_none());
    }

    #[test]
    fn normalized_egg_and_heart_match_requested_sampled_bounds() {
        let egg_bounds = bounding_box(&egg(Real::from(3_u8), Real::from(5_u8), 24));
        assert_eq!(
            egg_bounds.maxs.x.clone() - egg_bounds.mins.x,
            Real::from(3_u8)
        );
        assert_eq!(
            egg_bounds.maxs.y.clone() - egg_bounds.mins.y,
            Real::from(5_u8)
        );

        let heart_bounds = bounding_box(&heart(Real::from(4_u8), Real::from(6_u8), 32));
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
        );
        let involute_25 = involute_gear(
            Real::one(),
            16,
            Real::from(25_u8),
            Real::zero(),
            Real::zero(),
            4,
        );
        assert!(!involute_20.is_empty());
        assert!(!involute_25.is_empty());
        assert_ne!(involute_20, involute_25);

        let cycloidal_a = cycloidal_gear(Real::one(), 16, Real::one(), Real::zero(), 4);
        let cycloidal_b = cycloidal_gear(
            Real::one(),
            16,
            (Real::from(5_u8) / Real::from(4_u8)).unwrap(),
            Real::zero(),
            4,
        );
        assert!(!cycloidal_a.is_empty());
        assert!(!cycloidal_b.is_empty());
        assert_ne!(cycloidal_a, cycloidal_b);

        let rack_20 =
            involute_rack(Real::one(), 4, Real::from(20_u8), Real::zero(), Real::zero());
        let rack_25 =
            involute_rack(Real::one(), 4, Real::from(25_u8), Real::zero(), Real::zero());
        assert!(!rack_20.is_empty());
        assert!(!rack_25.is_empty());
        assert_ne!(rack_20, rack_25);
        assert_ne!(
            cycloidal_rack(Real::one(), 4, Real::zero(), 4),
            cycloidal_rack(Real::one(), 4, Real::zero(), 8)
        );

        let symmetric =
            airfoil_naca4(Real::zero(), Real::zero(), Real::from(12_u8), Real::one(), 20);
        let cambered = airfoil_naca4(
            Real::from(2_u8),
            Real::from(4_u8),
            Real::from(12_u8),
            Real::one(),
            20,
        );
        assert!(!symmetric.is_empty());
        assert!(!cambered.is_empty());
        assert_ne!(symmetric, cambered);
    }

    #[test]
    fn gear_and_rack_profiles_form_one_hole_free_material_component() {
        let cycloidal = cycloidal_gear(
            Real::one(),
            16,
            (Real::from(5_u8) / Real::from(4_u8)).unwrap(),
            Real::zero(),
            8,
        );
        let involute =
            involute_rack(Real::one(), 6, Real::from(20_u8), Real::zero(), Real::zero());
        let cycloidal_rack_profile = cycloidal_rack(Real::one(), 6, Real::zero(), 12);
        let readme_cycloidal = cycloidal_gear(
            Real::try_from(0.22).unwrap(),
            14,
            Real::try_from(0.275).unwrap(),
            Real::try_from(0.02).unwrap(),
            8,
        );
        assert!(
            cycloidal_gear(
                Real::one(),
                16,
                (Real::from(3_u8) / Real::from(4_u8)).unwrap(),
                Real::zero(),
                8,
            )
            .is_empty(),
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
            let profiles = match region
                .project_to_finite_profiles_exact(&projection, &CurvePolicy::STRICT)
                .unwrap_or_else(|error| panic!("{name} profile projection failed: {error}"))
            {
                Classification::Decided(profiles) => profiles,
                Classification::Uncertain(reason) => {
                    panic!("{name} profile topology is uncertain: {reason:?}")
                },
            };
            assert_eq!(
                profiles.len(),
                1,
                "{name} must be one connected material component"
            );
            assert!(
                profiles[0].holes().is_empty(),
                "{name} must not acquire holes from a self-intersecting boundary"
            );
            let paths = match region
                .project_to_finite_curve_paths(&CurvePolicy::STRICT)
                .unwrap_or_else(|error| panic!("{name} edge projection failed: {error}"))
            {
                Classification::Decided(paths) => paths,
                Classification::Uncertain(reason) => {
                    panic!("{name} edge topology is uncertain: {reason:?}")
                },
            };
            assert_eq!(
                paths.len(),
                1,
                "{name} must expose one simple closed edge path"
            );
        }

        for (name, rack) in [
            (
                "involute rack",
                involute_rack(Real::one(), 6, Real::from(20_u8), Real::zero(), Real::zero()),
            ),
            (
                "cycloidal rack",
                cycloidal_rack(Real::one(), 6, Real::zero(), 12),
            ),
        ] {
            let bounds = try_bounding_box(&rack)
                .unwrap_or_else(|error| panic!("{name} bounds failed: {error}"));
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
        assert!(bezier_path(&controls, 16).is_some());
        assert!(bspline_path(&controls, 2, 8).is_some());
        assert!(bezier_path(&controls, 0).is_none());
        assert!(bspline_path(&controls, 2, 0).is_none());

        let closed_controls = [
            [zero.clone(), zero.clone()],
            [two.clone(), zero.clone()],
            [two.clone(), two.clone()],
            [zero.clone(), zero],
        ];
        assert!(!bezier_region(&closed_controls, 16).is_empty());

        let mesh = try_triangulate(&square(two), &GeometryContext::STRICT)
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
        let compound = |result: Result<GeometryOutcome<CurveRegion2>, CurveBooleanError>| {
            result.expect("nominal compound region").into_value()
        };
        assert!(
            crescent(
                two.clone(),
                one.clone(),
                one.clone(),
                16,
                &GeometryContext::STRICT,
            )
            .is_err(),
            "strict compound construction must retain its symbolic boundary blocker"
        );
        let approximate_crescent = crescent(
            two.clone(),
            one.clone(),
            one.clone(),
            16,
            &GeometryContext::APPROXIMATE_512,
        )
        .expect("the authorized terminal should complete the sampled crescent");
        assert_eq!(
            approximate_crescent.certainty,
            crate::GeometryCertainty::Approximate512Consumed
        );
        assert!(!approximate_crescent.value.is_empty());
        assert!(matches!(
            try_finite_profiles(
                &approximate_crescent.value,
                &GeometryContext::APPROXIMATE_512,
            ),
            Err(ValidationError::Geometry(message))
                if message.contains("Unsupported")
        ));
        let shapes = [
            rectangle(two.clone(), one.clone()),
            square(two.clone()),
            circle(one.clone(), 16),
            right_triangle(two.clone(), one.clone()),
            polygon(&[
                [Real::zero(), Real::zero()],
                [one.clone(), Real::zero()],
                [Real::zero(), one.clone()],
            ]),
            ellipse(two.clone(), one.clone(), 16),
            regular_ngon(5, one.clone()),
            arrow(two.clone(), one.clone(), one.clone(), two.clone()),
            trapezoid(one.clone(), two.clone(), one.clone(), one.clone()),
            star(5, two.clone(), one.clone()),
            teardrop(two.clone(), Real::from(3_u8), 12),
            egg(two.clone(), Real::from(3_u8), 16),
            rounded_rectangle(two.clone(), one.clone(), three_quarters.clone(), 4),
            squircle(two.clone(), one.clone(), 16),
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
            pie_slice(one.clone(), Real::zero(), Real::from(90_u8), 8),
            heart(two.clone(), two.clone(), 16),
            supershape(
                one.clone(),
                one.clone(),
                Real::from(5_u8),
                two.clone(),
                one.clone(),
                one.clone(),
                24,
            ),
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
            ),
            cycloidal_gear(one.clone(), 16, one.clone(), Real::zero(), 4),
            involute_rack(one.clone(), 4, Real::from(20_u8), Real::zero(), Real::zero()),
            cycloidal_rack(one.clone(), 4, Real::zero(), 8),
            airfoil_naca4(Real::from(2_u8), Real::from(4_u8), Real::from(12_u8), one, 20),
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
        let strings = hilbert_strings(&square(Real::from(8_u8)), 3, Real::one());
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
}
