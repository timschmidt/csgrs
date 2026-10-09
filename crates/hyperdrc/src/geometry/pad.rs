//! Native pad outlines, constructed before any finite report projection.

mod report;

use super::{LineString, Polygon, exact_real};
use crate::{PcbGeometryUncertainty, Scalar};
use hypercurve::{CircularArc2, Contour2, CurveRegion2, LineSeg2, Point2, Segment2};

type NativeResult = Result<CurveRegion2, PcbGeometryUncertainty>;

/// Construct a KiCad trapezoid from exact imports of finite source parameters.
///
/// `delta` is the full difference between opposing edge lengths. Its X value
/// changes the vertical sides; its Y value changes the horizontal sides. Each
/// delta is halved before applying the corner offsets. A delta larger than the
/// opposite size is invalid; equality can produce a triangle. Signed sizes
/// denote their absolute values. A zero size denotes an empty regularized set.
///
/// Native lines, rotation and translation precede the finite report view.
/// Invalid input, unavailable geometry and shared-budget exhaustion remain
/// attached failures, observable through checked polygon queries.
pub fn trapezoid_polygon(
    center: [f64; 2],
    size: [f64; 2],
    delta: [f64; 2],
    angle_degrees: f64,
) -> Polygon<f64> {
    if !finite(center, size, angle_degrees) || !delta.into_iter().all(f64::is_finite) {
        return failed("trapezoid contains a non-finite parameter");
    }
    let native = placed(center, angle_degrees, || {
        let [x, y] = halves(size);
        let [dx, dy] = delta.map(|value| crate::scalar::half(&exact_real(value)));
        if crate::scalar::try_compare(&dx.abs(), &y)?.is_gt()
            || crate::scalar::try_compare(&dy.abs(), &x)?.is_gt()
        {
            return Err(failure("trapezoid delta exceeds the opposite size"));
        }
        if size.contains(&0.0) {
            return Ok(CurveRegion2::empty());
        }
        lines(vec![
            Point2::new(-&x - &dy, &y + &dx),
            Point2::new(&x + &dy, &y - &dx),
            Point2::new(&x - &dy, -&y + &dx),
            Point2::new(-x + dy, -y - dx),
        ])
    });
    finish(
        native,
        report::trapezoid(center, size, delta, angle_degrees),
    )
}

/// Construct a native rounded rectangle with exact lines and circular arcs.
///
/// Finite center, dimensions, radius and degrees are imported before arithmetic.
/// The radius is clamped to zero through half the shorter absolute dimension;
/// zero radius gives a rectangle and zero dimension gives a certified empty
/// region. Maximum radius gives an exact capsule or circle. Rotation and
/// translation stay native, including when finite report corners collapse.
///
/// `segments_per_corner` controls only the report view, with at least four
/// samples per quarter arc. Neither sampling nor a missing finite view decides
/// geometry. Checked queries retain construction and shared-budget failures.
pub fn rounded_rect_polygon(
    center: [f64; 2],
    size: [f64; 2],
    radius: f64,
    angle_degrees: f64,
    segments_per_corner: usize,
) -> Polygon<f64> {
    if !finite(center, size, angle_degrees) || !radius.is_finite() {
        return failed("rounded rectangle contains a non-finite parameter");
    }
    let native = placed(center, angle_degrees, || {
        if size.contains(&0.0) {
            return Ok(CurveRegion2::empty());
        }
        let [x, y] = halves(size);
        let radius = clamped_distance(radius, &x, &y)?;
        if radius == Scalar::zero() {
            return rectangle(x, y);
        }
        let corners = [
            (
                Point2::new(&x - &radius, -&y + &radius),
                Point2::new(&x - &radius, -y.clone()),
                Point2::new(x.clone(), -&y + &radius),
            ),
            (
                Point2::new(&x - &radius, &y - &radius),
                Point2::new(x.clone(), &y - &radius),
                Point2::new(&x - &radius, y.clone()),
            ),
            (
                Point2::new(-&x + &radius, &y - &radius),
                Point2::new(-&x + &radius, y.clone()),
                Point2::new(-x.clone(), &y - &radius),
            ),
            (
                Point2::new(-&x + &radius, -&y + &radius),
                Point2::new(-x.clone(), -&y + &radius),
                Point2::new(-x + &radius, -y),
            ),
        ];
        let mut segments = Vec::with_capacity(8);
        for (index, (center, start, end)) in corners.iter().enumerate() {
            segments.push(Segment2::Arc(
                CircularArc2::try_from_center(start.clone(), end.clone(), center.clone(), false)
                    .map_err(failure)?,
            ));
            let next = &corners[(index + 1) % corners.len()].1;
            if end != next {
                segments.push(Segment2::Line(
                    LineSeg2::try_new(end.clone(), next.clone()).map_err(failure)?,
                ));
            }
        }
        contour(segments)
    });
    finish(
        native,
        report::rounded(center, size, radius, angle_degrees, segments_per_corner),
    )
}

/// Construct a native rectangle with selected straight corner chamfers.
///
/// Corners are top-left, top-right, bottom-right, bottom-left in the pad's local
/// drawing coordinates. `chamfer` is an absolute distance, clamped to zero
/// through half the shorter absolute dimension. Exact imports precede clamping,
/// corner arithmetic and rotation, including for subnormal sizes. Zero size is
/// certified empty; no selected corner gives an ordinary rectangle. Repeated
/// vertices at a maximum chamfer do not create zero-length native edges.
/// Report corners are separate from the authoritative region; checked queries
/// retain construction and shared-budget failures.
pub fn chamfered_rect_polygon(
    center: [f64; 2],
    size: [f64; 2],
    chamfer: f64,
    corners: [bool; 4],
    angle_degrees: f64,
) -> Polygon<f64> {
    if !finite(center, size, angle_degrees) || !chamfer.is_finite() {
        return failed("chamfered rectangle contains a non-finite parameter");
    }
    let native = placed(center, angle_degrees, || {
        if size.contains(&0.0) {
            return Ok(CurveRegion2::empty());
        }
        let [x, y] = halves(size);
        let distance = clamped_distance(chamfer, &x, &y)?;
        if distance == Scalar::zero() || !corners.contains(&true) {
            return rectangle(x, y);
        }
        let [top_left, top_right, bottom_right, bottom_left] = corners;
        let mut points = vec![Point2::new(
            if top_left {
                -&x + &distance
            } else {
                -x.clone()
            },
            -y.clone(),
        )];
        if top_right {
            points.push(Point2::new(&x - &distance, -y.clone()));
            points.push(Point2::new(x.clone(), -&y + &distance));
        } else {
            points.push(Point2::new(x.clone(), -y.clone()));
        }
        if bottom_right {
            points.push(Point2::new(x.clone(), &y - &distance));
            points.push(Point2::new(&x - &distance, y.clone()));
        } else {
            points.push(Point2::new(x.clone(), y.clone()));
        }
        if bottom_left {
            points.push(Point2::new(-&x + &distance, y.clone()));
            points.push(Point2::new(-x.clone(), &y - &distance));
        } else {
            points.push(Point2::new(-x.clone(), y.clone()));
        }
        if top_left {
            points.push(Point2::new(-x, -y + distance));
        }
        lines(points)
    });
    finish(
        native,
        report::chamfered(center, size, chamfer, corners, angle_degrees),
    )
}

fn finite(center: [f64; 2], size: [f64; 2], angle: f64) -> bool {
    center
        .into_iter()
        .chain(size)
        .chain([angle])
        .all(f64::is_finite)
}

fn halves(size: [f64; 2]) -> [Scalar; 2] {
    size.map(|side| crate::scalar::half(&exact_real(side.abs())))
}

fn clamped_distance(value: f64, x: &Scalar, y: &Scalar) -> Result<Scalar, PcbGeometryUncertainty> {
    let value = exact_real(value.max(0.0));
    let limit = if crate::scalar::try_compare(x, y)?.is_le() {
        x
    } else {
        y
    };
    Ok(if crate::scalar::try_compare(&value, limit)?.is_gt() {
        limit.clone()
    } else {
        value
    })
}

fn rectangle(x: Scalar, y: Scalar) -> NativeResult {
    lines(vec![
        Point2::new(-x.clone(), -y.clone()),
        Point2::new(x.clone(), -y.clone()),
        Point2::new(x.clone(), y.clone()),
        Point2::new(-x, y),
    ])
}

fn lines(mut points: Vec<Point2>) -> NativeResult {
    points.dedup();
    if points.first() == points.last() {
        points.pop();
    }
    let edges = points
        .iter()
        .cloned()
        .zip(points.iter().cloned().cycle().skip(1))
        .map(|(a, b)| LineSeg2::try_new(a, b).map(Segment2::Line).map_err(failure))
        .collect::<Result<_, _>>()?;
    contour(edges)
}

fn contour(segments: Vec<Segment2>) -> NativeResult {
    let contour = Contour2::try_new(segments).map_err(failure)?;
    CurveRegion2::try_from_native_material_contours(vec![contour]).map_err(failure)
}

fn placed(
    center: [f64; 2],
    angle_degrees: f64,
    local: impl FnOnce() -> NativeResult,
) -> NativeResult {
    crate::operations::evaluate("pad-primitive-construction", None, || {
        let local = local()?;
        if local.is_empty() {
            return Ok(local);
        }
        let angle = (exact_real(angle_degrees) * Scalar::pi() / Scalar::from(180_u16))
            .expect("180 is a nonzero exact degree denominator");
        let cosine = angle.clone().cos();
        let sine = angle.sin();
        local
            .transform_affine(
                &cosine,
                &(-sine.clone()),
                &sine,
                &cosine,
                &exact_real(center[0]),
                &exact_real(center[1]),
            )
            .map_err(failure)
    })
}

fn finish(native: NativeResult, view: LineString<f64>) -> Polygon<f64> {
    match native {
        Ok(region) => Polygon::from_exact_region(region, view, vec![]),
        Err(error) => Polygon::from_exact_region(CurveRegion2::empty(), view, vec![])
            .with_exact_construction_error(error.to_string()),
    }
}

fn failure(detail: impl std::fmt::Display) -> PcbGeometryUncertainty {
    crate::operations::failure("pad-primitive-construction", None, detail)
}

fn failed(detail: &str) -> Polygon<f64> {
    finish(Err(failure(detail)), LineString(vec![]))
}
