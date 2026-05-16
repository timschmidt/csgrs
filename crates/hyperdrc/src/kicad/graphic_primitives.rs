//! Shared KiCad graphical primitive geometry.
//!
//! KiCad reuses rectangle, circle, and polygon graphics in custom pads and
//! footprint copper. This module keeps fill/stroke interpretation in one place
//! so those parser paths do not drift apart.

use geo::Polygon;

use crate::geometry::{arc_line_polygons, circle_polygon, line_polygon, polygon_from_points};
use crate::sexp::Sexp;

/// Return whether a KiCad graphic primitive should be treated as filled.
///
/// KiCad S-expressions encode explicit fill state as `(fill yes)`, `(fill no)`,
/// or newer symbolic values such as `solid`. Parser callers pass their current
/// conservative default for legacy or omitted fill fields.
pub(super) fn fill_enabled(item: &Sexp, default_filled: bool) -> bool {
    item.named_child("fill")
        .and_then(|fill| fill.atom_at(1))
        .map(|fill| matches!(fill, "yes" | "solid" | "true"))
        .unwrap_or(default_filled)
}

/// Build filled or stroked rectangle graphics.
///
/// Explicitly unfilled KiCad graphics are converted to four stroked edge
/// polygons instead of a solid rectangle. The edge construction follows the
/// same planar polygon model surveyed by Lee and Preparata, "Computational
/// Geometry - A Survey", IEEE Transactions on Computers, 1984,
/// <https://doi.org/10.1109/TC.1984.1676388>.
pub(super) fn rect_polygons(
    start: [f64; 2],
    end: [f64; 2],
    width: f64,
    filled: bool,
) -> Vec<Polygon<f64>> {
    let corners = [start, [end[0], start[1]], end, [start[0], end[1]]];
    if filled {
        return vec![polygon_from_points(corners.to_vec())];
    }

    closed_stroke_polygons(&corners, width)
}

/// Build filled or stroked circle graphics.
///
/// Stroked circles are sampled as short chord polygons. This is intentionally a
/// readiness approximation, not a lossless circular-arc data structure.
pub(super) fn circle_polygons(
    center: [f64; 2],
    end: [f64; 2],
    width: f64,
    filled: bool,
    segments: usize,
) -> Vec<Polygon<f64>> {
    let radius = distance(center, end);
    if filled {
        return vec![circle_polygon(center, radius, segments.max(16))];
    }
    if radius <= 0.0 {
        return Vec::new();
    }

    arc_line_polygons(
        center,
        [center[0] + radius, center[1]],
        360.0,
        width,
        segments.max(16),
    )
}

/// Build filled or stroked polygon graphics from a KiCad `(pts ...)` list.
pub(super) fn polygon_polygons(points: &[[f64; 2]], width: f64, filled: bool) -> Vec<Polygon<f64>> {
    if filled {
        if points.len() < 3 {
            return Vec::new();
        }
        return vec![polygon_from_points(points.to_vec())];
    }

    closed_stroke_polygons(points, width)
}

fn closed_stroke_polygons(points: &[[f64; 2]], width: f64) -> Vec<Polygon<f64>> {
    if points.len() < 2 || width <= 0.0 || !width.is_finite() {
        return Vec::new();
    }

    let mut polygons = points
        .windows(2)
        .filter_map(|edge| line_polygon(edge[0], edge[1], width))
        .collect::<Vec<_>>();

    if points.len() >= 3
        && let (Some(first), Some(last)) = (points.first(), points.last())
        && first != last
        && let Some(closing) = line_polygon(*last, *first, width)
    {
        polygons.push(closing);
    }

    polygons
}

fn distance(start: [f64; 2], end: [f64; 2]) -> f64 {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    (dx * dx + dy * dy).sqrt()
}
