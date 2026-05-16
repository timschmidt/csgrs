//! Primitive polygon constructors.
//!
//! These functions intentionally return plain `geo` polygons. Higher-level
//! modules decide whether to combine them into sketches or report shapes.
//! f64 is used here as parser/report edge geometry while `csgrs` remains
//! unported; topology-sensitive consumers should treat these polygons as
//! compatibility inputs, not the long-term semantic numeric core.

use csgrs::float_types::Real;
use geo::{Coord, LineString, Polygon};
use std::f64::consts::PI;

/// Run the `circle_polygon` design-readiness check or report helper.
pub fn circle_polygon(center: [f64; 2], radius: f64, segments: usize) -> Polygon<Real> {
    if !(center[0].is_finite() && center[1].is_finite() && radius.is_finite()) {
        return Polygon::new(geo::LineString(vec![]), vec![]);
    }

    let segments = segments.max(16);
    let mut coords = Vec::with_capacity(segments + 1);
    for index in 0..segments {
        let theta = 2.0 * PI * (index as f64) / (segments as f64);
        coords.push(Coord {
            x: center[0] + radius * theta.cos(),
            y: center[1] + radius * theta.sin(),
        });
    }
    coords.push(coords[0]);
    Polygon::new(LineString(coords), vec![])
}

/// Run the `rect_polygon` design-readiness check or report helper.
pub fn rect_polygon(center: [f64; 2], size: [f64; 2], angle_degrees: f64) -> Polygon<Real> {
    if !(center[0].is_finite()
        && center[1].is_finite()
        && size[0].is_finite()
        && size[1].is_finite()
        && angle_degrees.is_finite())
    {
        return polygon_from_points(Vec::new());
    }

    let half_x = size[0] / 2.0;
    let half_y = size[1] / 2.0;
    // Structural-dispatch note: parsed PCB primitives often carry exact grid,
    // rotation, axis-alignment, and pad-shape facts. Preserve those alongside
    // the future hyperreal geometry so downstream clearance and boolean checks
    // can select rectangle, rounded-rectangle, or affine-specialized kernels
    // instead of rediscovering the shape from sampled f64 vertices.
    let points = [
        [-half_x, -half_y],
        [half_x, -half_y],
        [half_x, half_y],
        [-half_x, half_y],
    ];
    polygon_from_points(
        points
            .iter()
            .map(|point| rotate_translate(*point, center, angle_degrees))
            .collect(),
    )
}

/// Build a KiCad-style trapezoid pad polygon.
///
/// KiCad's S-expression pad list includes `trapezoid` as a primitive pad
/// shape, with legacy libraries commonly carrying a `rect_delta` skew. The
/// point construction below mirrors KiCad's corner-offset convention: the X
/// delta shifts top and bottom edges oppositely, while the Y delta shifts left
/// and right edges oppositely. The resulting four-point polygon is intentionally
/// simple so downstream checks can use the same boolean path as rectangular
/// pads.
///
/// The affine corner construction follows the same polygonal geometry model
/// surveyed by Lee and Preparata, "Computational Geometry - A Survey",
/// IEEE Transactions on Computers, 1984, <https://doi.org/10.1109/TC.1984.1676388>.
pub fn trapezoid_polygon(
    center: [f64; 2],
    size: [f64; 2],
    delta: [f64; 2],
    angle_degrees: f64,
) -> Polygon<Real> {
    if !(center[0].is_finite()
        && center[1].is_finite()
        && size[0].is_finite()
        && size[1].is_finite()
        && delta[0].is_finite()
        && delta[1].is_finite()
        && angle_degrees.is_finite())
    {
        return polygon_from_points(Vec::new());
    }

    let half_x = size[0].abs() / 2.0;
    let half_y = size[1].abs() / 2.0;
    let dx = delta[0];
    let dy = delta[1];
    let points = [
        [-half_x - dy, half_y + dx],
        [half_x + dy, half_y - dx],
        [half_x - dy, -half_y - dx],
        [-half_x + dy, -half_y + dx],
    ];

    polygon_from_points(
        points
            .iter()
            .map(|point| rotate_translate(*point, center, angle_degrees))
            .collect(),
    )
}

/// Build a KiCad-style rounded rectangle as a single closed polygon.
///
/// `radius` is interpreted as an absolute corner radius in the same coordinate
/// units as `size`. The value is clamped to half of the shorter side, matching
/// the land-pattern constraint used by rounded SMT pads. The corner arcs are a
/// deterministic polygonal approximation; downstream checks can then use the
/// same `geo`/`csgrs` boolean pipeline as rectangular and circular pads.
///
/// The offset-region construction follows the computational-geometry framing
/// surveyed by Lee and Preparata, "Computational Geometry - A Survey",
/// IEEE Transactions on Computers, 1984, <https://doi.org/10.1109/TC.1984.1676388>.
pub fn rounded_rect_polygon(
    center: [f64; 2],
    size: [f64; 2],
    radius: f64,
    angle_degrees: f64,
    segments_per_corner: usize,
) -> Polygon<Real> {
    if !(center[0].is_finite()
        && center[1].is_finite()
        && size[0].is_finite()
        && size[1].is_finite()
        && radius.is_finite()
        && angle_degrees.is_finite())
    {
        return polygon_from_points(Vec::new());
    }

    let width = size[0].abs();
    let height = size[1].abs();
    let max_radius = width.min(height) / 2.0;
    let radius = radius.clamp(0.0, max_radius);
    if radius <= 0.0 || width == 0.0 || height == 0.0 {
        return rect_polygon(center, size, angle_degrees);
    }

    let half_x = width / 2.0;
    let half_y = height / 2.0;
    let segments = segments_per_corner.max(4);
    let corners = [
        ([half_x - radius, -half_y + radius], -90.0, 0.0),
        ([half_x - radius, half_y - radius], 0.0, 90.0),
        ([-half_x + radius, half_y - radius], 90.0, 180.0),
        ([-half_x + radius, -half_y + radius], 180.0, 270.0),
    ];

    let mut points = Vec::with_capacity(corners.len() * (segments + 1));
    for (corner_center, start_degrees, end_degrees) in corners {
        for step in 0..=segments {
            let t = step as f64 / segments as f64;
            let theta = (start_degrees + (end_degrees - start_degrees) * t).to_radians();
            let point = [
                corner_center[0] + radius * theta.cos(),
                corner_center[1] + radius * theta.sin(),
            ];
            points.push(rotate_translate(point, center, angle_degrees));
        }
    }

    polygon_from_points(points)
}

/// Build a rectangle with selected chamfered corners.
///
/// KiCad represents chamfered pad corners with a chamfer ratio and a corner
/// list. Callers pass the already-resolved absolute chamfer distance plus
/// corners ordered as top-left, top-right, bottom-right, bottom-left. The
/// returned polygon uses straight chamfer segments and falls back to a plain
/// rectangle when no corner is selected.
///
/// This is a polygon clipping specialization of the planar geometry operations
/// surveyed by Lee and Preparata, "Computational Geometry - A Survey",
/// IEEE Transactions on Computers, 1984, <https://doi.org/10.1109/TC.1984.1676388>.
pub fn chamfered_rect_polygon(
    center: [f64; 2],
    size: [f64; 2],
    chamfer: f64,
    corners: [bool; 4],
    angle_degrees: f64,
) -> Polygon<Real> {
    if !(center[0].is_finite()
        && center[1].is_finite()
        && size[0].is_finite()
        && size[1].is_finite()
        && chamfer.is_finite()
        && angle_degrees.is_finite())
    {
        return polygon_from_points(Vec::new());
    }

    let width = size[0].abs();
    let height = size[1].abs();
    let chamfer = chamfer.clamp(0.0, width.min(height) / 2.0);
    if chamfer <= 0.0 || !corners.iter().any(|selected| *selected) {
        return rect_polygon(center, size, angle_degrees);
    }

    let half_x = width / 2.0;
    let half_y = height / 2.0;
    let [top_left, top_right, bottom_right, bottom_left] = corners;
    let mut points = Vec::with_capacity(8);

    if top_left {
        points.push([-half_x + chamfer, -half_y]);
    } else {
        points.push([-half_x, -half_y]);
    }

    if top_right {
        points.push([half_x - chamfer, -half_y]);
        points.push([half_x, -half_y + chamfer]);
    } else {
        points.push([half_x, -half_y]);
    }

    if bottom_right {
        points.push([half_x, half_y - chamfer]);
        points.push([half_x - chamfer, half_y]);
    } else {
        points.push([half_x, half_y]);
    }

    if bottom_left {
        points.push([-half_x + chamfer, half_y]);
        points.push([-half_x, half_y - chamfer]);
    } else {
        points.push([-half_x, half_y]);
    }

    if top_left {
        points.push([-half_x, -half_y + chamfer]);
    }

    polygon_from_points(
        points
            .into_iter()
            .map(|point| rotate_translate(point, center, angle_degrees))
            .collect(),
    )
}

/// Run the `line_polygon` design-readiness check or report helper.
pub fn line_polygon(start: [f64; 2], end: [f64; 2], width: f64) -> Option<Polygon<Real>> {
    if !all_finite(start) || !all_finite(end) || !width.is_finite() {
        return None;
    }

    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let length = (dx * dx + dy * dy).sqrt();
    if length == 0.0 || width <= 0.0 {
        return None;
    }

    let nx = -dy / length * width / 2.0;
    let ny = dx / length * width / 2.0;
    Some(polygon_from_points(vec![
        [start[0] + nx, start[1] + ny],
        [end[0] + nx, end[1] + ny],
        [end[0] - nx, end[1] - ny],
        [start[0] - nx, start[1] - ny],
    ]))
}

/// Run the `polygon_from_points` design-readiness check or report helper.
pub fn polygon_from_points(points: Vec<[f64; 2]>) -> Polygon<Real> {
    if points
        .iter()
        .any(|point| !point[0].is_finite() || !point[1].is_finite())
    {
        return Polygon::new(LineString(vec![]), vec![]);
    }

    let mut coords = points
        .into_iter()
        .map(|point| Coord {
            x: point[0],
            y: point[1],
        })
        .collect::<Vec<_>>();

    if coords.first() != coords.last()
        && let Some(first) = coords.first().copied()
    {
        coords.push(first);
    }

    Polygon::new(LineString(coords), vec![])
}

/// Run the `transform_polygon` design-readiness check or report helper.
pub fn transform_polygon(
    polygon: &Polygon<Real>,
    origin: [f64; 2],
    angle_degrees: f64,
) -> Polygon<Real> {
    if !origin[0].is_finite() || !origin[1].is_finite() || !angle_degrees.is_finite() {
        return polygon.clone();
    }

    let exterior = polygon
        .exterior()
        .0
        .iter()
        .map(|coord| rotate_translate([coord.x, coord.y], origin, angle_degrees))
        .collect::<Vec<_>>();
    let holes = polygon
        .interiors()
        .iter()
        .map(|ring| {
            LineString(
                ring.0
                    .iter()
                    .map(|coord| {
                        let point = rotate_translate([coord.x, coord.y], origin, angle_degrees);
                        Coord {
                            x: point[0],
                            y: point[1],
                        }
                    })
                    .collect(),
            )
        })
        .collect();

    Polygon::new(
        LineString(
            exterior
                .into_iter()
                .map(|point| Coord {
                    x: point[0],
                    y: point[1],
                })
                .collect(),
        ),
        holes,
    )
}

/// Run the `arc_line_polygons` design-readiness check or report helper.
pub fn arc_line_polygons(
    center: [f64; 2],
    start: [f64; 2],
    angle_degrees: f64,
    width: f64,
    segments: usize,
) -> Vec<Polygon<Real>> {
    if !all_finite([center[0], center[1]])
        || !all_finite([start[0], start[1]])
        || !width.is_finite()
        || !angle_degrees.is_finite()
    {
        return Vec::new();
    }

    let segments = segments.max(4);
    let mut points = Vec::with_capacity(segments + 1);
    let start_vector = [start[0] - center[0], start[1] - center[1]];

    for index in 0..=segments {
        let theta = angle_degrees.to_radians() * (index as f64) / (segments as f64);
        let cos = theta.cos();
        let sin = theta.sin();
        points.push([
            center[0] + start_vector[0] * cos - start_vector[1] * sin,
            center[1] + start_vector[0] * sin + start_vector[1] * cos,
        ]);
    }

    points
        .windows(2)
        .filter_map(|window| line_polygon(window[0], window[1], width))
        .collect()
}

/// Approximate cubic Bezier curves as stroked line polygons.
///
/// KiCad stores board and footprint curve graphics as four-point cubic Bezier
/// curves. This helper samples each complete cubic span, then turns each
/// adjacent sample pair into the same rectangular stroke polygons used for
/// traces and graphical lines. When callers pass a longer point list, connected
/// spans are read as `p0,p1,p2,p3`, then `p3,p4,p5,p6`, matching the common
/// contiguous cubic-path convention while still accepting KiCad's documented
/// four-point form.
///
/// The evaluation uses de Casteljau subdivision, the stable geometric
/// construction presented in Farin, *Curves and Surfaces for CAGD: A Practical
/// Guide*, 5th ed., Academic Press, 2002.
pub fn bezier_line_polygons(
    points: &[[f64; 2]],
    width: f64,
    segments: usize,
) -> Vec<Polygon<Real>> {
    if points.len() < 4
        || points.iter().any(|point| !all_finite(*point))
        || !width.is_finite()
        || width <= 0.0
    {
        return Vec::new();
    }

    let segments = segments.max(4);
    let mut polygons = Vec::new();
    let mut index = 0;
    while index + 3 < points.len() {
        let control = [
            points[index],
            points[index + 1],
            points[index + 2],
            points[index + 3],
        ];
        let samples = (0..=segments)
            .map(|step| {
                let t = step as f64 / segments as f64;
                cubic_bezier_point(control, t)
            })
            .collect::<Vec<_>>();
        polygons.extend(
            samples
                .windows(2)
                .filter_map(|window| line_polygon(window[0], window[1], width)),
        );
        index += 3;
    }

    polygons
}

fn cubic_bezier_point(control: [[f64; 2]; 4], t: f64) -> [f64; 2] {
    let a = lerp_point(control[0], control[1], t);
    let b = lerp_point(control[1], control[2], t);
    let c = lerp_point(control[2], control[3], t);
    let d = lerp_point(a, b, t);
    let e = lerp_point(b, c, t);
    lerp_point(d, e, t)
}

fn lerp_point(start: [f64; 2], end: [f64; 2], t: f64) -> [f64; 2] {
    [
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
    ]
}

fn all_finite(point: [f64; 2]) -> bool {
    point[0].is_finite() && point[1].is_finite()
}

fn rotate_translate(point: [f64; 2], center: [f64; 2], angle_degrees: f64) -> [f64; 2] {
    let theta = angle_degrees.to_radians();
    let cos = theta.cos();
    let sin = theta.sin();
    [
        center[0] + point[0] * cos - point[1] * sin,
        center[1] + point[0] * sin + point[1] * cos,
    ]
}
