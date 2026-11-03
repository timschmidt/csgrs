//! Exact geometric helpers shared by the PCB clearance modules.
//!
//! Each function here was previously copied, byte for byte, into several
//! `pcb_*` modules. Keeping one copy keeps their decisions identical.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals};
use hyperreal::Real;

use crate::pcb::{ClearanceStatus, PadBoardClearanceReport, TraceClearanceReport};
use crate::segment::LinePathSegment;

pub(crate) fn classify_margins(
    margins: &[Real; 4],
    required: &Real,
    policy: PredicatePolicy,
) -> Option<(ClearanceStatus, Real)> {
    let mut minimum_margin = margins[0].clone();
    for margin in margins {
        if matches!(
            compare_reals(margin, &minimum_margin, policy).value(),
            Some(Ordering::Less)
        ) {
            minimum_margin = margin.clone();
        }
        match compare_reals(margin, required, policy).value()? {
            Ordering::Less => return Some((ClearanceStatus::ClearanceViolation, margin.clone())),
            Ordering::Equal | Ordering::Greater => {}
        }
    }
    Some((ClearanceStatus::CertifiedClear, minimum_margin))
}

pub(crate) fn classify_swept_distance(
    distance_squared: Real,
    trace_width: &Real,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    let four_distance_squared = distance_squared * Real::from(4);
    let overlap_limit_squared = trace_width.clone() * trace_width.clone();
    let clearance_limit = trace_width.clone() + required_clearance.clone() * Real::from(2);
    let clearance_limit_squared = clearance_limit.clone() * clearance_limit;
    let status = match compare_reals(&four_distance_squared, &overlap_limit_squared, policy).value()
    {
        Some(Ordering::Less | Ordering::Equal) => ClearanceStatus::NoShortViolation,
        Some(Ordering::Greater) => {
            match compare_reals(&four_distance_squared, &clearance_limit_squared, policy).value() {
                Some(Ordering::Less) => ClearanceStatus::ClearanceViolation,
                Some(Ordering::Equal | Ordering::Greater) => ClearanceStatus::CertifiedClear,
                None => ClearanceStatus::Unknown,
            }
        }
        None => ClearanceStatus::Unknown,
    };
    TraceClearanceReport {
        status,
        centerline_intersection: None,
        axis_gap: None,
    }
}

pub(crate) fn cross(first: &Point2, second: &Point2) -> Real {
    first.x.clone() * second.y.clone() - first.y.clone() * second.x.clone()
}

pub(crate) fn dot(first: &Point2, second: &Point2) -> Real {
    Real::signed_product_sum([true, true], [[&first.x, &second.x], [&first.y, &second.y]])
}

pub(crate) fn edge_cross(a: &Point2, b: &Point2, c: &Point2) -> Real {
    let ab_x = b.x.clone() - a.x.clone();
    let ab_y = b.y.clone() - a.y.clone();
    let ac_x = c.x.clone() - a.x.clone();
    let ac_y = c.y.clone() - a.y.clone();
    ab_x * ac_y - ab_y * ac_x
}

pub(crate) fn point_segment_distance_squared(
    point: &Point2,
    segment: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<Real> {
    let ab = Point2::new(
        segment.end().x.clone() - segment.start().x.clone(),
        segment.end().y.clone() - segment.start().y.clone(),
    );
    let ap = Point2::new(
        point.x.clone() - segment.start().x.clone(),
        point.y.clone() - segment.start().y.clone(),
    );
    let bp = Point2::new(
        point.x.clone() - segment.end().x.clone(),
        point.y.clone() - segment.end().y.clone(),
    );
    let length_squared = squared_norm(&ab);
    match compare_reals(&length_squared, &Real::zero(), policy).value()? {
        Ordering::Equal => return Some(squared_norm(&ap)),
        Ordering::Less => return None,
        Ordering::Greater => {}
    }
    let projection = dot(&ap, &ab);
    if !matches!(
        compare_reals(&projection, &Real::zero(), policy).value()?,
        Ordering::Greater
    ) {
        return Some(squared_norm(&ap));
    }
    if !matches!(
        compare_reals(&projection, &length_squared, policy).value()?,
        Ordering::Less
    ) {
        return Some(squared_norm(&bp));
    }
    let cross_value = cross(&ap, &ab);
    let cross_squared = cross_value.clone() * cross_value;
    (cross_squared / length_squared).ok()
}

pub(crate) fn polygon_edges(
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Option<Vec<LinePathSegment>> {
    (0..vertices.len())
        .map(|index| {
            LinePathSegment::new(
                vertices[index].clone(),
                vertices[(index + 1) % vertices.len()].clone(),
                policy,
            )
            .ok()
        })
        .collect()
}

pub(crate) fn polygon_signed_area_twice(vertices: &[Point2]) -> Real {
    let mut area = Real::zero();
    for index in 0..vertices.len() {
        let current = &vertices[index];
        let next = &vertices[(index + 1) % vertices.len()];
        area = area + current.x.clone() * next.y.clone() - next.x.clone() * current.y.clone();
    }
    area
}

pub(crate) fn squared_norm(vector: &Point2) -> Real {
    Real::signed_product_sum(
        [true, true],
        [[&vector.x, &vector.x], [&vector.y, &vector.y]],
    )
}

pub(crate) fn unknown_pad_board_report() -> PadBoardClearanceReport {
    PadBoardClearanceReport {
        status: ClearanceStatus::Unknown,
        copper_gap: None,
    }
}

pub(crate) fn unknown_trace_report() -> TraceClearanceReport {
    TraceClearanceReport {
        status: ClearanceStatus::Unknown,
        centerline_intersection: None,
        axis_gap: None,
    }
}

pub(crate) fn update_minimum_distance(
    minimum: &mut Option<Real>,
    candidate: Real,
    policy: PredicatePolicy,
) -> Option<()> {
    let replace = match minimum.as_ref() {
        Some(current) => compare_reals(&candidate, current, policy).value()? == Ordering::Less,
        None => true,
    };
    if replace {
        *minimum = Some(candidate);
    }
    Some(())
}

pub(crate) fn vertex_extrema(
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Option<(Real, Real, Real, Real)> {
    let mut min_x = vertices[0].x.clone();
    let mut max_x = vertices[0].x.clone();
    let mut min_y = vertices[0].y.clone();
    let mut max_y = vertices[0].y.clone();
    for vertex in vertices.iter().skip(1) {
        if compare_reals(&vertex.x, &min_x, policy).value()? == Ordering::Less {
            min_x = vertex.x.clone();
        }
        if compare_reals(&vertex.x, &max_x, policy).value()? == Ordering::Greater {
            max_x = vertex.x.clone();
        }
        if compare_reals(&vertex.y, &min_y, policy).value()? == Ordering::Less {
            min_y = vertex.y.clone();
        }
        if compare_reals(&vertex.y, &max_y, policy).value()? == Ordering::Greater {
            max_y = vertex.y.clone();
        }
    }
    Some((min_x, max_x, min_y, max_y))
}

pub(crate) fn segment_segment_distance_squared(
    first: &LinePathSegment,
    second: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<Real> {
    let mut minimum = None;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(first.start(), second, policy)?,
        policy,
    )?;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(first.end(), second, policy)?,
        policy,
    )?;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(second.start(), first, policy)?,
        policy,
    )?;
    update_minimum_distance(
        &mut minimum,
        point_segment_distance_squared(second.end(), first, policy)?,
        policy,
    )?;
    minimum
}
