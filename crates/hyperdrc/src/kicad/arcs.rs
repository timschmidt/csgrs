//! KiCad arc reconstruction helpers.
//!
//! KiCad commonly stores arcs as start/mid/end triples. Keeping the
//! circumcircle math here lets board graphics and custom-pad primitives share
//! the same sweep convention.

use hyperlimit::{Point2, PredicatePolicy, Sign, orient2d_with_policy};

use crate::geometry::RuleGeometryProvenance;
#[cfg(test)]
use crate::geometry::{SourceGridFacts, SourceUnit};

use super::ParsedPoint2;

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
#[cfg(test)]
pub(super) fn arc_center_start_angle(
    start: [f64; 2],
    mid: [f64; 2],
    end: [f64; 2],
) -> Option<([f64; 2], [f64; 2], f64)> {
    if !all_finite(start) || !all_finite(mid) || !all_finite(end) {
        return None;
    }
    arc_center_start_angle_checked(start, mid, end, exact_arc_orientation(start, mid, end)?)
}

/// Recover an arc while retaining KiCad decimal-token exactness for degeneracy.
///
/// This is the parser-aware companion to [`arc_center_start_angle`]. The
/// returned center and sweep are still `f64` compatibility geometry for the
/// existing polygon stroker, but the collinearity decision consumes retained
/// decimal-token [`Real`](hyperreal::Real) values whenever possible. Yap's EGC
/// model treats this kind of representation preservation as part of choosing
/// the right arithmetic package for a geometric predicate; see Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).
pub(super) fn arc_center_start_angle_source(
    start: &ParsedPoint2,
    mid: &ParsedPoint2,
    end: &ParsedPoint2,
) -> Option<([f64; 2], [f64; 2], f64)> {
    if !all_finite(start.approximate)
        || !all_finite(mid.approximate)
        || !all_finite(end.approximate)
    {
        return None;
    }
    arc_center_start_angle_checked(
        start.approximate,
        mid.approximate,
        end.approximate,
        exact_arc_orientation_source(start, mid, end)?,
    )
}

fn arc_center_start_angle_checked(
    start: [f64; 2],
    mid: [f64; 2],
    end: [f64; 2],
    orientation: Sign,
) -> Option<([f64; 2], [f64; 2], f64)> {
    if orientation == Sign::Zero {
        return None;
    }

    let d = 2.0
        * (start[0] * (mid[1] - end[1])
            + mid[0] * (end[1] - start[1])
            + end[0] * (start[1] - mid[1]));
    if !d.is_finite() {
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

#[cfg(test)]
fn exact_arc_orientation(start: [f64; 2], mid: [f64; 2], end: [f64; 2]) -> Option<Sign> {
    // The circumcircle denominator is twice the orientation determinant of the
    // three arc points. Its zero/nonzero status controls whether a KiCad arc is
    // geometrically well-defined, so the degeneracy decision belongs to the
    // exact predicate layer rather than an f64 epsilon. This follows Yap's
    // exact geometric computation boundary; the subsequent center and angle are
    // still parser/rendering-edge approximations. See Yap, "Towards Exact
    // Geometric Computation," Computational Geometry 7.1-2 (1997), and Lee and
    // Preparata, "Computational Geometry - A Survey," IEEE Transactions on
    // Computers 33.12 (1984).
    //
    let provenance = RuleGeometryProvenance::new(
        "kicad-arc-orientation",
        SourceGridFacts::primitive_float_edge(SourceUnit::KiCadMillimeter),
    );
    let start = lift_point(start, provenance)?;
    let mid = lift_point(mid, provenance)?;
    let end = lift_point(end, provenance)?;
    orient2d_with_policy(&start, &mid, &end, PredicatePolicy::STRICT).value()
}

fn exact_arc_orientation_source(
    start: &ParsedPoint2,
    mid: &ParsedPoint2,
    end: &ParsedPoint2,
) -> Option<Sign> {
    let provenance = RuleGeometryProvenance::new(
        "kicad-arc-orientation",
        start.combined_grid(mid).combine(end.grid),
    );
    let start = exact_or_lift_point(start, provenance)?;
    let mid = exact_or_lift_point(mid, provenance)?;
    let end = exact_or_lift_point(end, provenance)?;
    orient2d_with_policy(&start, &mid, &end, PredicatePolicy::STRICT).value()
}

fn exact_or_lift_point(point: &ParsedPoint2, provenance: RuleGeometryProvenance) -> Option<Point2> {
    if let Some([x, y]) = &point.exact {
        return Some(Point2::new(x.clone(), y.clone()));
    }
    lift_point(point.approximate, provenance)
}

fn lift_point(point: [f64; 2], provenance: RuleGeometryProvenance) -> Option<Point2> {
    Some(Point2::new(
        provenance.lift_f64(point[0])?,
        provenance.lift_f64(point[1])?,
    ))
}

#[cfg(test)]
mod tests {
    use super::{arc_center_start_angle, arc_center_start_angle_source};
    use crate::kicad::xy_from_child_source;
    use crate::sexp;

    #[test]
    fn arc_center_start_angle_rejects_exactly_collinear_points() {
        assert!(arc_center_start_angle([0.0, 0.0], [1.0, 0.0], [2.0, 0.0]).is_none());
    }

    #[test]
    fn arc_center_start_angle_accepts_tiny_nonzero_orientation() {
        let arc = arc_center_start_angle([0.0, 0.0], [1.0, 1.0e-12], [2.0, 0.0])
            .expect("exact non-collinear arcs must not be rejected by an epsilon");

        assert!(arc.0[0].is_finite());
        assert!(arc.0[1].is_finite());
        assert!(arc.2.is_finite());
    }

    #[test]
    fn arc_center_start_angle_rejects_non_finite_inputs() {
        assert!(arc_center_start_angle([f64::NAN, 0.0], [1.0, 1.0], [2.0, 0.0]).is_none());
    }

    #[test]
    fn arc_center_start_angle_source_uses_decimal_token_exactness() {
        let parsed =
            sexp::parse("(gr_arc (start 0.0 0.0) (mid 1.0 0.000000000001) (end 2.0 0.0))").unwrap();
        let start = xy_from_child_source(&parsed, "start").unwrap();
        let mid = xy_from_child_source(&parsed, "mid").unwrap();
        let end = xy_from_child_source(&parsed, "end").unwrap();

        let arc = arc_center_start_angle_source(&start, &mid, &end)
            .expect("exact source-token orientation should keep a tiny nonzero arc");

        assert!(arc.0[0].is_finite());
        assert!(arc.0[1].is_finite());
        assert!(arc.2.is_finite());
    }
}
