//! Exact segment tracing for winding-number propagation.

use hyperlattice::{HomogeneousPoint3, Point3, Real};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{
    Aabb, Classification, Plane, axis_mut, axis_ref, classify_point, classify_real, compare_real,
};
use crate::polygon::ConvexPolygon;
use crate::winding::{WindingNumberTransitionVector, WindingNumberVector};

/// Result of tracing one axis-aligned segment.
#[derive(Clone, Debug, PartialEq)]
pub struct TraceAxisSegmentResult {
    /// Winding number after accepted crossings.
    pub winding: WindingNumberVector,
    /// Whether the path avoided exact edge hits.
    pub valid: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct CrossingEvent {
    point: Point3,
    support: Plane,
    normal_sign: i32,
    cross_sign: i32,
    delta_w: WindingNumberTransitionVector,
    on_edge: bool,
}

/// Traces an axis-aligned segment, accumulating polygon winding transitions.
pub fn trace_axis_segment(
    start: &Point3,
    end: &Point3,
    axis: usize,
    start_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<TraceAxisSegmentResult> {
    let mut winding = start_wnv.to_vec();
    let direction = compare_real(axis_ref(end, axis), axis_ref(start, axis))?;
    if direction.is_eq() {
        return Ok(TraceAxisSegmentResult {
            winding,
            valid: true,
        });
    }

    let dir_sign = if direction.is_gt() { 1 } else { -1 };
    let mut events = Vec::new();
    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let normal_axis = axis_ref(&polygon.support.normal, axis);
        if normal_axis.definitely_zero() {
            continue;
        }

        let Some(crossing) = segment_plane_crossing(start, end, &polygon.support)? else {
            continue;
        };

        if !point_strictly_between_axis(&crossing, start, end, axis)? {
            continue;
        }

        let mut inside = true;
        let mut on_edge = false;
        for edge in &polygon.edges {
            match classify_point(&crossing, edge)? {
                Classification::Positive => {
                    inside = false;
                    break;
                }
                Classification::On => on_edge = true,
                Classification::Negative => {}
            }
        }
        if !inside {
            continue;
        }

        let normal_sign = match crate::geometry::classify_real(normal_axis)? {
            Classification::Positive => 1,
            Classification::Negative => -1,
            Classification::On => continue,
        };
        let cross_sign = normal_sign * -dir_sign;
        events.push(CrossingEvent {
            point: crossing,
            support: polygon.support.clone(),
            normal_sign,
            cross_sign,
            delta_w: polygon.delta_w.clone(),
            on_edge,
        });
    }

    let mut accepted: Vec<CrossingEvent> = Vec::new();
    for (index, event) in events.iter().enumerate() {
        if event.on_edge
            && !events.iter().enumerate().any(|(other_index, other)| {
                other_index != index
                    && other.point == event.point
                    && other.support == event.support
                    && other.normal_sign == event.normal_sign
                    && other.delta_w == event.delta_w
            })
        {
            return Ok(TraceAxisSegmentResult {
                winding,
                valid: false,
            });
        }

        if accepted.iter().any(|existing| {
            existing.point == event.point
                && existing.support == event.support
                && existing.normal_sign == event.normal_sign
                && existing.delta_w == event.delta_w
        }) {
            continue;
        }

        accepted.push(event.clone());
    }

    sort_crossing_events(&mut accepted, axis, dir_sign)?;

    for event in accepted {
        for (value, delta) in winding.iter_mut().zip(&event.delta_w) {
            *value += event.cross_sign * *delta;
        }
    }

    Ok(TraceAxisSegmentResult {
        winding,
        valid: true,
    })
}

const AXIS_ORDERINGS: [[usize; 3]; 6] = [
    [0, 1, 2],
    [0, 2, 1],
    [1, 0, 2],
    [1, 2, 0],
    [2, 0, 1],
    [2, 1, 0],
];

/// Traces an axis-aligned polyline using several axis orderings and returns
/// the first valid winding result. If direct L-shaped paths are blocked by
/// exact surface hits, retries through arrangement-coordinate endpoint-box
/// detours.
pub fn trace_segment(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    if let Ok(winding) = trace_axis_ordered_paths(start, end, winding, polygons) {
        return Ok(winding);
    }

    for detour in interior_box_detour_points(start, end, polygons)? {
        if detour == *start || detour == *end || point_lies_on_traced_surface(&detour, polygons)? {
            continue;
        }
        let Ok(first_leg) = trace_axis_ordered_paths(start, &detour, winding, polygons) else {
            continue;
        };
        let Ok(second_leg) = trace_axis_ordered_paths(&detour, end, &first_leg, polygons) else {
            continue;
        };
        return Ok(second_leg);
    }

    Err(HypermeshError::UnknownClassification)
}

fn trace_axis_ordered_paths(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    for ordering in AXIS_ORDERINGS {
        let mut current = start.clone();
        let mut attempt = winding.to_vec();
        let mut valid = true;

        for axis in ordering {
            if compare_real(axis_ref(&current, axis), axis_ref(end, axis))?.is_ne() {
                let mut next = current.clone();
                *axis_mut(&mut next, axis) = axis_ref(end, axis).clone();
                if next != *end && point_lies_on_traced_surface(&next, polygons)? {
                    valid = false;
                    break;
                }
                let traced = trace_axis_segment(&current, &next, axis, &attempt, polygons)?;
                attempt = traced.winding;
                valid = traced.valid;
                current = next;
                if !valid {
                    break;
                }
            }
        }

        if valid {
            return Ok(attempt);
        }
    }

    Err(HypermeshError::UnknownClassification)
}

fn interior_box_detour_points(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<Point3>> {
    let mut values = vec![Vec::new(), Vec::new(), Vec::new()];
    for (axis, axis_values) in values.iter_mut().enumerate() {
        let start_value = axis_ref(start, axis);
        let end_value = axis_ref(end, axis);
        if compare_real(start_value, end_value)?.is_eq() {
            axis_values.push(start_value.clone());
            continue;
        }

        let mut cuts = Vec::new();
        push_unique_ordered_real(&mut cuts, start_value.clone())?;
        push_unique_ordered_real(&mut cuts, end_value.clone())?;
        for polygon in polygons {
            for vertex in polygon.vertices()? {
                let value = axis_ref(&vertex, axis);
                if value_strictly_between(value, start_value, end_value)? {
                    push_unique_ordered_real(&mut cuts, value.clone())?;
                }
            }
        }

        for endpoints in cuts.windows(2) {
            let midpoint = ((&endpoints[0] + &endpoints[1]) / Real::from(2))
                .map_err(|_| HypermeshError::UnknownClassification)?;
            axis_values.push(midpoint);
        }
    }

    let mut detours = Vec::with_capacity(values[0].len() * values[1].len() * values[2].len());
    for x in &values[0] {
        for y in &values[1] {
            for z in &values[2] {
                detours.push(Point3::new(x.clone(), y.clone(), z.clone()));
            }
        }
    }
    Ok(detours)
}

fn value_strictly_between(value: &Real, a: &Real, b: &Real) -> HypermeshResult<bool> {
    let value_to_a = compare_real(value, a)?;
    let value_to_b = compare_real(value, b)?;
    Ok((value_to_a.is_gt() && value_to_b.is_lt()) || (value_to_a.is_lt() && value_to_b.is_gt()))
}

fn push_unique_ordered_real(values: &mut Vec<Real>, value: Real) -> HypermeshResult<()> {
    for (index, existing) in values.iter().enumerate() {
        match compare_real(&value, existing)? {
            std::cmp::Ordering::Equal => return Ok(()),
            std::cmp::Ordering::Less => {
                values.insert(index, value);
                return Ok(());
            }
            std::cmp::Ordering::Greater => {}
        }
    }
    values.push(value);
    Ok(())
}

fn point_lies_on_traced_surface(
    point: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        if classify_point(point, &polygon.support)? != Classification::On {
            continue;
        }

        let mut inside_polygon = true;
        for edge in &polygon.edges {
            if classify_point(point, edge)? == Classification::Positive {
                inside_polygon = false;
                break;
            }
        }
        if inside_polygon {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Finds a probe point off a polygon surface and reports its side.
pub fn find_probe_point(
    polygon: &ConvexPolygon,
) -> HypermeshResult<Option<(Point3, Classification)>> {
    find_probe_point_on_side(polygon, true)
}

fn find_probe_point_on_side(
    polygon: &ConvexPolygon,
    positive_side: bool,
) -> HypermeshResult<Option<(Point3, Classification)>> {
    if polygon.vertex_count() < 3 {
        return Ok(None);
    }

    let vertices = polygon.vertices()?;
    let center = centroid(&vertices).ok_or(HypermeshError::EmptyInput)?;
    let axis = dominant_normal_axis(&polygon.support)?;
    let normal_sign = crate::geometry::classify_real(axis_ref(&polygon.support.normal, axis))?;
    if normal_sign == Classification::On {
        return Ok(None);
    }

    let offset = probe_offset(&vertices, axis)?;
    let mut probe = center;
    let signed_offset = if (normal_sign == Classification::Positive) == positive_side {
        offset
    } else {
        -offset
    };
    *axis_mut(&mut probe, axis) = axis_ref(&probe, axis) + &signed_offset;

    let mut side = classify_point(&probe, &polygon.support)?;
    if side == Classification::On {
        *axis_mut(&mut probe, axis) = axis_ref(&probe, axis) + &signed_offset;
        side = classify_point(&probe, &polygon.support)?;
    }

    if side == Classification::On {
        Ok(None)
    } else {
        Ok(Some((probe, side)))
    }
}

/// Classifies a leaf polygon by tracing from a reference point to an off-face
/// probe and applying the host transition correction.
pub fn classify_leaf_polygon(
    support: &Plane,
    leaf_edges: &[Plane],
    ref_point: &Point3,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
) -> HypermeshResult<WindingNumberVector> {
    let leaf = ConvexPolygon {
        support: support.clone(),
        edges: leaf_edges.to_vec(),
        mesh_index: -1,
        polygon_index: -1,
        delta_w: WindingNumberTransitionVector::new(),
        no_self_intersections: false,
        no_nested_components: false,
        approx_bounds: None,
    };

    let interior_points = interior_leaf_points(&leaf)?;
    for point in &interior_points {
        for positive_side in [true, false] {
            for (probe, probe_side) in
                bounded_probes_from_interior(point, support, bounds, positive_side, polygons)?
            {
                if point_lies_on_traced_surface(&probe, polygons)? {
                    continue;
                }
                if !probe_reaches_adjacent_cell(point, &probe, support, polygons)? {
                    continue;
                }
                let Ok(mut winding) = trace_segment(ref_point, &probe, ref_wnv, polygons) else {
                    continue;
                };
                if probe_side == Classification::Negative {
                    for (value, delta) in winding.iter_mut().zip(host_delta_w) {
                        *value -= *delta;
                    }
                }
                return Ok(winding);
            }
        }
    }

    Err(HypermeshError::UnknownClassification)
}

fn probe_reaches_adjacent_cell(
    start: &Point3,
    probe: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let Some(axis) = changed_axis(start, probe)? else {
        return Ok(false);
    };

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let start_class = classify_point(start, &polygon.support)?;
        let probe_class = classify_point(probe, &polygon.support)?;

        if start_class == Classification::On {
            if planes_are_coplanar(&polygon.support, host_support)? {
                continue;
            }
            if point_lies_on_polygon(start, polygon)? {
                return Ok(false);
            }
            continue;
        }

        if probe_class == Classification::On {
            if point_lies_on_polygon(probe, polygon)? {
                return Ok(false);
            }
            continue;
        }

        if start_class == probe_class {
            continue;
        }

        let Some(crossing) = segment_plane_crossing(start, probe, &polygon.support)? else {
            continue;
        };
        if point_strictly_between_axis(&crossing, start, probe, axis)?
            && point_lies_on_polygon(&crossing, polygon)?
        {
            return Ok(false);
        }
    }

    Ok(true)
}

fn planes_are_coplanar(left: &Plane, right: &Plane) -> HypermeshResult<bool> {
    let left_coefficients = [&left.normal.x, &left.normal.y, &left.normal.z, &left.offset];
    let right_coefficients = [
        &right.normal.x,
        &right.normal.y,
        &right.normal.z,
        &right.offset,
    ];

    for i in 0..left_coefficients.len() {
        for j in (i + 1)..left_coefficients.len() {
            let determinant = (left_coefficients[i] * right_coefficients[j])
                - (left_coefficients[j] * right_coefficients[i]);
            if classify_real(&determinant)? != Classification::On {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

fn changed_axis(start: &Point3, end: &Point3) -> HypermeshResult<Option<usize>> {
    let mut changed = None;
    for axis in 0..3 {
        if compare_real(axis_ref(start, axis), axis_ref(end, axis))?.is_ne() {
            if changed.is_some() {
                return Ok(None);
            }
            changed = Some(axis);
        }
    }
    Ok(changed)
}

fn point_lies_on_polygon(point: &Point3, polygon: &ConvexPolygon) -> HypermeshResult<bool> {
    if classify_point(point, &polygon.support)? != Classification::On {
        return Ok(false);
    }
    for edge in &polygon.edges {
        if classify_point(point, edge)? == Classification::Positive {
            return Ok(false);
        }
    }
    Ok(true)
}

fn segment_plane_crossing(
    start: &Point3,
    end: &Point3,
    plane: &Plane,
) -> HypermeshResult<Option<Point3>> {
    let start_value = plane.expression_at_point(start);
    let end_value = plane.expression_at_point(end);
    let start_class = crate::geometry::classify_real(&start_value)?;
    let end_class = crate::geometry::classify_real(&end_value)?;

    if start_class == Classification::On || end_class == Classification::On {
        return Ok(None);
    }
    if start_class == end_class {
        return Ok(None);
    }

    let denom = &start_value - &end_value;
    let t = (start_value / denom).map_err(|_| HypermeshError::UnknownClassification)?;
    Ok(Some(Point3::new(
        &start.x + &(t.clone() * (&end.x - &start.x)),
        &start.y + &(t.clone() * (&end.y - &start.y)),
        &start.z + &(t * (&end.z - &start.z)),
    )))
}

fn point_strictly_between_axis(
    point: &Point3,
    start: &Point3,
    end: &Point3,
    axis: usize,
) -> HypermeshResult<bool> {
    let start_to_point = compare_real(axis_ref(point, axis), axis_ref(start, axis))?;
    let point_to_end = compare_real(axis_ref(point, axis), axis_ref(end, axis))?;
    Ok((start_to_point.is_gt() && point_to_end.is_lt())
        || (start_to_point.is_lt() && point_to_end.is_gt()))
}

fn sort_crossing_events(
    events: &mut Vec<CrossingEvent>,
    axis: usize,
    dir_sign: i32,
) -> HypermeshResult<()> {
    let mut sorted: Vec<CrossingEvent> = Vec::with_capacity(events.len());
    for event in events.drain(..) {
        let mut insert_at = sorted.len();
        for (index, existing) in sorted.iter().enumerate() {
            let order = compare_real(
                axis_ref(&event.point, axis),
                axis_ref(&existing.point, axis),
            )?;
            if (dir_sign > 0 && order.is_lt()) || (dir_sign < 0 && order.is_gt()) {
                insert_at = index;
                break;
            }
        }
        sorted.insert(insert_at, event);
    }
    *events = sorted;
    Ok(())
}

fn dominant_normal_axis(plane: &Plane) -> HypermeshResult<usize> {
    let abs = [
        plane.normal.x.clone().abs(),
        plane.normal.y.clone().abs(),
        plane.normal.z.clone().abs(),
    ];
    let mut best = 0;
    for axis in 1..3 {
        if compare_real(&abs[axis], &abs[best])?.is_gt() {
            best = axis;
        }
    }
    Ok(best)
}

fn centroid(points: &[Point3]) -> Option<Point3> {
    if points.is_empty() {
        return None;
    }
    let mut sum = Point3::origin();
    for point in points {
        sum.x += point.x.clone();
        sum.y += point.y.clone();
        sum.z += point.z.clone();
    }
    let denom = Real::from(points.len() as u64);
    Some(Point3::new(
        (sum.x / denom.clone()).expect("point count is non-zero"),
        (sum.y / denom.clone()).expect("point count is non-zero"),
        (sum.z / denom).expect("point count is non-zero"),
    ))
}

fn interior_leaf_points(leaf: &ConvexPolygon) -> HypermeshResult<Vec<Point3>> {
    let vertices = leaf.vertices()?;
    let Some(center) = centroid(&vertices) else {
        return Ok(Vec::new());
    };

    let mut points = Vec::with_capacity(vertices.len() + 1);
    if point_strictly_inside_leaf(&center, leaf)? {
        points.push(center.clone());
    }

    let n = Real::from(vertices.len() as u64);
    let denom = &n + &Real::one();
    for vertex in vertices {
        let candidate = Point3::new(
            (((&center.x * &n) + vertex.x) / denom.clone())
                .map_err(|_| HypermeshError::UnknownClassification)?,
            (((&center.y * &n) + vertex.y) / denom.clone())
                .map_err(|_| HypermeshError::UnknownClassification)?,
            (((&center.z * &n) + vertex.z) / denom.clone())
                .map_err(|_| HypermeshError::UnknownClassification)?,
        );
        if point_strictly_inside_leaf(&candidate, leaf)? {
            points.push(candidate);
        }
    }

    Ok(points)
}

fn point_strictly_inside_leaf(point: &Point3, leaf: &ConvexPolygon) -> HypermeshResult<bool> {
    let homogeneous = HomogeneousPoint3::new(
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
        Real::one(),
    );
    leaf.contains_point_strictly(&homogeneous)
}

fn bounded_probes_from_interior(
    interior: &Point3,
    support: &Plane,
    bounds: &Aabb,
    positive_side: bool,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<(Point3, Classification)>> {
    let mut probes = Vec::new();

    for axis in probe_axes(support)? {
        let normal_sign = crate::geometry::classify_real(axis_ref(&support.normal, axis))?;
        if normal_sign == Classification::On {
            continue;
        }

        let direction_positive = (normal_sign == Classification::Positive) == positive_side;
        let axis_value = axis_ref(interior, axis);
        let room = if direction_positive {
            axis_ref(&bounds.max, axis) - axis_value
        } else {
            axis_value - axis_ref(&bounds.min, axis)
        };
        if !compare_real(&room, &Real::zero())?.is_gt() {
            continue;
        }

        let Some(probe) =
            adjacent_axis_probe(interior, bounds, polygons, axis, direction_positive)?
        else {
            continue;
        };

        let side = classify_point(&probe, support)?;
        if side != Classification::On
            && !probes
                .iter()
                .any(|(existing, _): &(Point3, Classification)| existing == &probe)
        {
            probes.push((probe, side));
        }
    }

    Ok(probes)
}

fn adjacent_axis_probe(
    interior: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
) -> HypermeshResult<Option<Point3>> {
    let start_value = axis_ref(interior, axis);
    let bound_value = if direction_positive {
        axis_ref(&bounds.max, axis)
    } else {
        axis_ref(&bounds.min, axis)
    };
    if !axis_value_after_start(start_value, bound_value, direction_positive)? {
        return Ok(None);
    }

    let mut endpoint = interior.clone();
    *axis_mut(&mut endpoint, axis) = bound_value.clone();
    let mut stop_value = bound_value.clone();

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let Some(crossing) = segment_plane_crossing(interior, &endpoint, &polygon.support)? else {
            continue;
        };
        if !point_strictly_between_axis(&crossing, interior, &endpoint, axis)? {
            continue;
        }
        if !point_lies_on_polygon(&crossing, polygon)? {
            continue;
        }

        let crossing_value = axis_ref(&crossing, axis);
        if axis_value_after_start(start_value, crossing_value, direction_positive)?
            && axis_value_before_stop(crossing_value, &stop_value, direction_positive)?
        {
            stop_value = crossing_value.clone();
        }
    }

    if !axis_value_after_start(start_value, &stop_value, direction_positive)? {
        return Ok(None);
    }

    let midpoint = ((start_value + &stop_value) / Real::from(2))
        .map_err(|_| HypermeshError::UnknownClassification)?;
    let mut probe = interior.clone();
    *axis_mut(&mut probe, axis) = midpoint;
    Ok(Some(probe))
}

fn axis_value_after_start(
    start: &Real,
    value: &Real,
    direction_positive: bool,
) -> HypermeshResult<bool> {
    let order = compare_real(value, start)?;
    Ok((direction_positive && order.is_gt()) || (!direction_positive && order.is_lt()))
}

fn axis_value_before_stop(
    value: &Real,
    stop: &Real,
    direction_positive: bool,
) -> HypermeshResult<bool> {
    let order = compare_real(value, stop)?;
    Ok((direction_positive && order.is_lt()) || (!direction_positive && order.is_gt()))
}

fn probe_axes(support: &Plane) -> HypermeshResult<Vec<usize>> {
    let dominant = dominant_normal_axis(support)?;
    let mut axes = vec![dominant];
    for axis in 0..3 {
        if axis != dominant && !axis_ref(&support.normal, axis).definitely_zero() {
            axes.push(axis);
        }
    }
    Ok(axes)
}

fn probe_offset(points: &[Point3], axis: usize) -> HypermeshResult<Real> {
    let mut min = axis_ref(&points[0], axis).clone();
    let mut max = min.clone();
    for point in &points[1..] {
        if compare_real(axis_ref(point, axis), &min)?.is_lt() {
            min = axis_ref(point, axis).clone();
        }
        if compare_real(axis_ref(point, axis), &max)?.is_gt() {
            max = axis_ref(point, axis).clone();
        }
    }
    let extent = max - min;
    if extent.definitely_zero() {
        Ok(Real::one())
    } else {
        Ok(
            (extent.abs() / Real::from(10)).expect("division by literal ten is valid")
                + Real::one(),
        )
    }
}
