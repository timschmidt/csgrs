//! Finite drawing views; none of these coordinates return to native geometry.

use super::super::{Coord, LineString};

pub(super) fn trapezoid(
    center: [f64; 2],
    size: [f64; 2],
    delta: [f64; 2],
    angle: f64,
) -> LineString<f64> {
    let [x, y] = size.map(|side| side.abs() / 2.0);
    let [dx, dy] = delta.map(|value| value / 2.0);
    ring(
        vec![
            [-x - dy, y + dx],
            [x + dy, y - dx],
            [x - dy, -y + dx],
            [-x + dy, -y - dx],
        ],
        center,
        angle,
    )
}

pub(super) fn rounded(
    center: [f64; 2],
    size: [f64; 2],
    radius: f64,
    angle: f64,
    samples: usize,
) -> LineString<f64> {
    let [width, height] = size.map(f64::abs);
    let radius = radius.clamp(0.0, width.min(height) / 2.0);
    if radius <= 0.0 || width == 0.0 || height == 0.0 {
        return rectangle(center, size, angle);
    }
    let x = width / 2.0;
    let y = height / 2.0;
    let samples = samples.max(4);
    let corners = [
        ([x - radius, -y + radius], -90.0, 0.0),
        ([x - radius, y - radius], 0.0, 90.0),
        ([-x + radius, y - radius], 90.0, 180.0),
        ([-x + radius, -y + radius], 180.0, 270.0),
    ];
    let mut points = Vec::with_capacity(4 * (samples + 1));
    for (corner, start, end) in corners {
        for step in 0..=samples {
            let t = step as f64 / samples as f64;
            let theta = (start + (end - start) * t).to_radians();
            points.push([
                corner[0] + radius * theta.cos(),
                corner[1] + radius * theta.sin(),
            ]);
        }
    }
    ring(points, center, angle)
}

pub(super) fn chamfered(
    center: [f64; 2],
    size: [f64; 2],
    distance: f64,
    corners: [bool; 4],
    angle: f64,
) -> LineString<f64> {
    let [width, height] = size.map(f64::abs);
    let distance = distance.clamp(0.0, width.min(height) / 2.0);
    if distance <= 0.0 || !corners.contains(&true) {
        return rectangle(center, size, angle);
    }
    let x = width / 2.0;
    let y = height / 2.0;
    let [tl, tr, br, bl] = corners;
    let mut points = vec![[if tl { -x + distance } else { -x }, -y]];
    if tr {
        points.extend([[x - distance, -y], [x, -y + distance]]);
    } else {
        points.push([x, -y]);
    }
    if br {
        points.extend([[x, y - distance], [x - distance, y]]);
    } else {
        points.push([x, y]);
    }
    if bl {
        points.extend([[-x + distance, y], [-x, y - distance]]);
    } else {
        points.push([-x, y]);
    }
    if tl {
        points.push([-x, -y + distance]);
    }
    ring(points, center, angle)
}

fn rectangle(center: [f64; 2], size: [f64; 2], angle: f64) -> LineString<f64> {
    let [x, y] = size.map(|side| side / 2.0);
    ring(vec![[-x, -y], [x, -y], [x, y], [-x, y]], center, angle)
}

fn ring(points: Vec<[f64; 2]>, center: [f64; 2], angle: f64) -> LineString<f64> {
    let angle = angle.to_radians();
    let cosine = angle.cos();
    let sine = angle.sin();
    let mut points = points
        .into_iter()
        .map(|[x, y]| Coord {
            x: center[0] + x * cosine - y * sine,
            y: center[1] + x * sine + y * cosine,
        })
        .collect::<Vec<_>>();
    if points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
        return LineString(vec![]);
    }
    if points.first() != points.last()
        && let Some(first) = points.first().copied()
    {
        points.push(first);
    }
    LineString(points)
}
