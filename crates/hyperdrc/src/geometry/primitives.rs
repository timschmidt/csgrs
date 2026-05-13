//! Primitive polygon constructors.
//!
//! These functions intentionally return plain `geo` polygons. Higher-level
//! modules decide whether to combine them into sketches or report shapes.

use csgrs::float_types::Real;
use geo::{Coord, LineString, Polygon};
use std::f64::consts::PI;

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
