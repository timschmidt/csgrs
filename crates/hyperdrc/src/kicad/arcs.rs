//! KiCad arc reconstruction helpers.
//!
//! KiCad commonly stores arcs as start/mid/end triples. Keeping the
//! circumcircle math here lets board graphics and custom-pad primitives share
//! the same sweep convention.

/// Recover a circle center, start point, and signed sweep angle from three arc points.
///
/// The midpoint decides whether the shorter counter-clockwise sweep or the
/// complementary clockwise sweep represents the KiCad arc. This is the same
/// circumcircle construction used by the board-graphics parser; degenerate or
/// non-finite triples return `None` so callers can skip malformed graphics
/// without synthesizing fallback geometry.
///
/// This helper follows the computational-geometry treatment of circular arcs
/// and orientation predicates surveyed by Lee and Preparata, "Computational
/// Geometry - A Survey", IEEE Transactions on Computers, 1984,
/// <https://doi.org/10.1109/TC.1984.1676388>.
pub(super) fn arc_center_start_angle(
    start: [f64; 2],
    mid: [f64; 2],
    end: [f64; 2],
) -> Option<([f64; 2], [f64; 2], f64)> {
    if !all_finite(start) || !all_finite(mid) || !all_finite(end) {
        return None;
    }

    let d = 2.0
        * (start[0] * (mid[1] - end[1])
            + mid[0] * (end[1] - start[1])
            + end[0] * (start[1] - mid[1]));
    if !d.is_finite() || d.abs() < 1.0e-9 {
        return None;
    }

    let start_sq = start[0] * start[0] + start[1] * start[1];
    let mid_sq = mid[0] * mid[0] + mid[1] * mid[1];
    let end_sq = end[0] * end[0] + end[1] * end[1];
    let center = [
        (start_sq * (mid[1] - end[1])
            + mid_sq * (end[1] - start[1])
            + end_sq * (start[1] - mid[1]))
            / d,
        (start_sq * (end[0] - mid[0])
            + mid_sq * (start[0] - end[0])
            + end_sq * (mid[0] - start[0]))
            / d,
    ];
    if !all_finite(center) {
        return None;
    }

    let start_angle = (start[1] - center[1]).atan2(start[0] - center[0]);
    let mid_angle = (mid[1] - center[1]).atan2(mid[0] - center[0]);
    let end_angle = (end[1] - center[1]).atan2(end[0] - center[0]);
    let ccw_delta = positive_angle_delta(start_angle, end_angle);
    let mid_delta = positive_angle_delta(start_angle, mid_angle);
    let angle = if mid_delta <= ccw_delta {
        ccw_delta.to_degrees()
    } else {
        -(std::f64::consts::TAU - ccw_delta).to_degrees()
    };

    angle.is_finite().then_some((center, start, angle))
}

fn positive_angle_delta(start: f64, end: f64) -> f64 {
    let mut delta = end - start;
    while delta < 0.0 {
        delta += std::f64::consts::TAU;
    }
    while delta >= std::f64::consts::TAU {
        delta -= std::f64::consts::TAU;
    }
    delta
}

fn all_finite(point: [f64; 2]) -> bool {
    point[0].is_finite() && point[1].is_finite()
}
