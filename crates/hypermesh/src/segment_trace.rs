//! Exact segment tracing for winding-number propagation.

use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};

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

#[derive(Clone, Debug, PartialEq)]
struct PlaneDefinedPoint {
    planes: [Plane; 3],
}

#[derive(Clone, Debug, PartialEq)]
struct InteriorLeafPoint {
    point: Point3,
    planes: Option<[Plane; 3]>,
}

#[derive(Clone, Debug, PartialEq)]
struct ProbePoint {
    point: Point3,
    side: Classification,
    planes: Option<[Plane; 3]>,
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

    let Some(mut accepted) = accepted_crossing_events(&events) else {
        return Ok(TraceAxisSegmentResult {
            winding,
            valid: false,
        });
    };

    sort_crossing_events(&mut accepted, axis, dir_sign)?;

    for event in accepted {
        apply_winding_transition_in_place(&mut winding, event.cross_sign, &event.delta_w)?;
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
    if let Some(winding) = retryable_trace(trace_axis_ordered_paths(start, end, winding, polygons))?
    {
        return Ok(winding);
    }

    if let Some(traced) = retryable_trace(trace_direct_segment(start, end, winding, polygons))?
        && traced.valid
    {
        return Ok(traced.winding);
    }

    for detour in interior_box_detour_points(start, end, polygons)? {
        if detour == *start || detour == *end || point_lies_on_traced_surface(&detour, polygons)? {
            continue;
        }
        let Some(first_leg) =
            retryable_trace(trace_axis_ordered_paths(start, &detour, winding, polygons))?
        else {
            continue;
        };
        let Some(second_leg) =
            retryable_trace(trace_axis_ordered_paths(&detour, end, &first_leg, polygons))?
        else {
            continue;
        };
        return Ok(second_leg);
    }

    Err(HypermeshError::UnknownClassification)
}

pub(crate) fn trace_plane_replacement_path(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    for ordering in AXIS_ORDERINGS {
        let mut current_planes = start_planes.clone();
        let mut current_point = match affine_from_planes(&current_planes) {
            Ok(point) => point,
            Err(HypermeshError::UnknownClassification) => continue,
            Err(err) => return Err(err),
        };
        let mut attempt = winding.to_vec();
        let mut valid = true;

        for plane_index in ordering {
            current_planes[plane_index] = end_planes[plane_index].clone();
            let next_point = match affine_from_planes(&current_planes) {
                Ok(point) => point,
                Err(HypermeshError::UnknownClassification) => {
                    valid = false;
                    break;
                }
                Err(err) => return Err(err),
            };
            if next_point != current_point {
                let Some(next_winding) = retryable_trace(trace_segment(
                    &current_point,
                    &next_point,
                    &attempt,
                    polygons,
                ))?
                else {
                    valid = false;
                    break;
                };
                attempt = next_winding;
                current_point = next_point;
            }
        }

        if valid {
            return Ok(attempt);
        }
    }

    Err(HypermeshError::UnknownClassification)
}

pub(crate) fn affine_from_planes(planes: &[Plane; 3]) -> HypermeshResult<Point3> {
    intersect_three_planes(&planes[0], &planes[1], &planes[2])
        .to_affine_point()
        .map_err(|_| HypermeshError::UnknownClassification)
}

pub(crate) fn axis_plane_definition(point: &Point3) -> [Plane; 3] {
    [
        Plane::axis_aligned(0, point.x.clone()),
        Plane::axis_aligned(1, point.y.clone()),
        Plane::axis_aligned(2, point.z.clone()),
    ]
}

fn axis_plane_defined_point(point: &Point3) -> PlaneDefinedPoint {
    PlaneDefinedPoint {
        planes: axis_plane_definition(point),
    }
}

fn retryable_trace<T>(result: HypermeshResult<T>) -> HypermeshResult<Option<T>> {
    match result {
        Ok(value) => Ok(Some(value)),
        Err(HypermeshError::UnknownClassification) => Ok(None),
        Err(err) => Err(err),
    }
}

fn apply_winding_transition_in_place(
    winding: &mut [i32],
    sign: i32,
    delta_w: &[i32],
) -> HypermeshResult<()> {
    if winding.len() != delta_w.len() {
        return Err(HypermeshError::UnknownClassification);
    }
    for (value, delta) in winding.iter_mut().zip(delta_w) {
        *value += sign * *delta;
    }
    Ok(())
}

fn trace_direct_segment(
    start: &Point3,
    end: &Point3,
    start_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<TraceAxisSegmentResult> {
    let mut winding = start_wnv.to_vec();
    let Some(sort_axis) = first_changed_axis(start, end)? else {
        return Ok(TraceAxisSegmentResult {
            winding,
            valid: true,
        });
    };
    let dir_sign = if compare_real(axis_ref(end, sort_axis), axis_ref(start, sort_axis))?.is_gt() {
        1
    } else {
        -1
    };

    let mut events = Vec::new();
    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let start_value = polygon.support.expression_at_point(start);
        let end_value = polygon.support.expression_at_point(end);
        let start_class = classify_real(&start_value)?;
        let end_class = classify_real(&end_value)?;
        if start_class == Classification::On || end_class == Classification::On {
            continue;
        }
        if start_class == end_class {
            continue;
        }

        let Some(crossing) = segment_plane_crossing(start, end, &polygon.support)? else {
            continue;
        };

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

        let normal_axis = dominant_normal_axis(&polygon.support)?;
        let normal_sign = match classify_real(axis_ref(&polygon.support.normal, normal_axis))? {
            Classification::Positive => 1,
            Classification::Negative => -1,
            Classification::On => continue,
        };
        let cross_sign = match classify_real(&(&start_value - &end_value))? {
            Classification::Positive => 1,
            Classification::Negative => -1,
            Classification::On => continue,
        };
        events.push(CrossingEvent {
            point: crossing,
            support: polygon.support.clone(),
            normal_sign,
            cross_sign,
            delta_w: polygon.delta_w.clone(),
            on_edge,
        });
    }

    let Some(mut accepted) = accepted_crossing_events(&events) else {
        return Ok(TraceAxisSegmentResult {
            winding,
            valid: false,
        });
    };
    sort_crossing_events(&mut accepted, sort_axis, dir_sign)?;

    for event in accepted {
        apply_winding_transition_in_place(&mut winding, event.cross_sign, &event.delta_w)?;
    }

    Ok(TraceAxisSegmentResult {
        winding,
        valid: true,
    })
}

fn accepted_crossing_events(events: &[CrossingEvent]) -> Option<Vec<CrossingEvent>> {
    let mut accepted = Vec::new();
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
            return None;
        }

        if accepted.iter().any(|existing: &CrossingEvent| {
            existing.point == event.point
                && existing.support == event.support
                && existing.normal_sign == event.normal_sign
                && existing.delta_w == event.delta_w
        }) {
            continue;
        }

        accepted.push(event.clone());
    }
    Some(accepted)
}

fn first_changed_axis(start: &Point3, end: &Point3) -> HypermeshResult<Option<usize>> {
    for axis in 0..3 {
        if compare_real(axis_ref(start, axis), axis_ref(end, axis))?.is_ne() {
            return Ok(Some(axis));
        }
    }
    Ok(None)
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
            add_axis_box_surface_cuts(
                &mut cuts,
                start,
                end,
                polygon,
                axis,
                start_value,
                end_value,
            )?;
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

fn add_axis_box_surface_cuts(
    cuts: &mut Vec<Real>,
    start: &Point3,
    end: &Point3,
    polygon: &ConvexPolygon,
    axis: usize,
    start_value: &Real,
    end_value: &Real,
) -> HypermeshResult<()> {
    let other_axes = other_axes(axis);
    for first in [axis_ref(start, other_axes[0]), axis_ref(end, other_axes[0])] {
        for second in [axis_ref(start, other_axes[1]), axis_ref(end, other_axes[1])] {
            let mut edge_start = Point3::origin();
            let mut edge_end = Point3::origin();
            *axis_mut(&mut edge_start, axis) = start_value.clone();
            *axis_mut(&mut edge_end, axis) = end_value.clone();
            *axis_mut(&mut edge_start, other_axes[0]) = first.clone();
            *axis_mut(&mut edge_end, other_axes[0]) = first.clone();
            *axis_mut(&mut edge_start, other_axes[1]) = second.clone();
            *axis_mut(&mut edge_end, other_axes[1]) = second.clone();

            let Some(crossing) = segment_plane_crossing(&edge_start, &edge_end, &polygon.support)?
            else {
                continue;
            };
            if point_lies_on_polygon(&crossing, polygon)? {
                let value = axis_ref(&crossing, axis);
                if value_strictly_between(value, start_value, end_value)? {
                    push_unique_ordered_real(cuts, value.clone())?;
                }
            }
        }
    }
    Ok(())
}

fn other_axes(axis: usize) -> [usize; 2] {
    match axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => unreachable!("axis must be in 0..3"),
    }
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

/// Classifies a leaf polygon by tracing from a reference point to an off-face
/// probe and applying the host transition correction.
pub fn classify_leaf_polygon(
    support: &Plane,
    leaf_edges: &[Plane],
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
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
        approx_bounds: None,
    };

    let interior_points = interior_leaf_points(&leaf)?;
    for point in &interior_points {
        for positive_side in [true, false] {
            for probe in
                bounded_probes_from_interior(point, support, bounds, positive_side, polygons)?
            {
                if point_lies_on_traced_surface(&probe.point, polygons)? {
                    continue;
                }
                if !probe_reaches_adjacent_cell(&point.point, &probe.point, support, polygons)? {
                    continue;
                }
                let mut winding =
                    retryable_trace(trace_segment(ref_point, &probe.point, ref_wnv, polygons))?;
                if winding.is_none()
                    && let Some(probe_planes) = &probe.planes
                {
                    winding = retryable_trace(trace_probe_from_reference_definitions(
                        ref_point,
                        ref_definitions,
                        probe_planes,
                        ref_wnv,
                        polygons,
                    ))?;
                };
                let Some(mut winding) = winding else {
                    continue;
                };
                if probe.side == Classification::Negative {
                    apply_winding_transition_in_place(&mut winding, -1, host_delta_w)?;
                }
                return Ok(winding);
            }
        }
    }

    Err(HypermeshError::UnknownClassification)
}

fn trace_probe_from_reference_definitions(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    probe_planes: &[Plane; 3],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    if ref_definitions.is_empty() {
        return trace_plane_replacement_path(
            &axis_plane_defined_point(ref_point).planes,
            probe_planes,
            ref_wnv,
            polygons,
        );
    }

    for start_definition in ref_definitions {
        match trace_plane_replacement_path(start_definition, probe_planes, ref_wnv, polygons) {
            Ok(winding) => return Ok(winding),
            Err(HypermeshError::UnknownClassification) => continue,
            Err(err) => return Err(err),
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
    let Some(sort_axis) = first_changed_axis(start, probe)? else {
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
        if point_strictly_between_axis(&crossing, start, probe, sort_axis)?
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

fn centroid(points: &[Point3]) -> HypermeshResult<Option<Point3>> {
    if points.is_empty() {
        return Ok(None);
    }
    let mut sum = Point3::origin();
    for point in points {
        sum.x += point.x.clone();
        sum.y += point.y.clone();
        sum.z += point.z.clone();
    }
    let denom = Real::from(points.len() as u64);
    Ok(Some(Point3::new(
        (sum.x / denom.clone()).map_err(|_| HypermeshError::UnknownClassification)?,
        (sum.y / denom.clone()).map_err(|_| HypermeshError::UnknownClassification)?,
        (sum.z / denom).map_err(|_| HypermeshError::UnknownClassification)?,
    )))
}

fn interior_leaf_points(leaf: &ConvexPolygon) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let vertices = leaf.vertices()?;
    let Some(center) = centroid(&vertices)? else {
        return Ok(Vec::new());
    };

    let mut points = Vec::with_capacity(vertices.len() + 1);
    if point_strictly_inside_leaf(&center, leaf)? {
        points.push(InteriorLeafPoint {
            point: center.clone(),
            planes: None,
        });
        for candidate in shifted_edge_interior_points(leaf, &center)? {
            push_unique_interior_point(&mut points, candidate);
        }
    }

    Ok(points)
}

fn shifted_edge_interior_points(
    leaf: &ConvexPolygon,
    strict_interior: &Point3,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let mut points = Vec::with_capacity(leaf.vertex_count());
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;

    for first_edge in 0..leaf.vertex_count() {
        let second_edge = (first_edge + 1) % leaf.vertex_count();
        let first_margin = leaf.edges[first_edge].expression_at_point(strict_interior);
        let second_margin = leaf.edges[second_edge].expression_at_point(strict_interior);
        if classify_real(&first_margin)? != Classification::Negative
            || classify_real(&second_margin)? != Classification::Negative
        {
            continue;
        }

        let first_shifted =
            inward_shifted_edge_plane(&leaf.edges[first_edge], &first_margin, &half);
        let second_shifted =
            inward_shifted_edge_plane(&leaf.edges[second_edge], &second_margin, &half);
        let candidate = intersect_three_planes(&leaf.support, &first_shifted, &second_shifted)
            .to_affine_point()
            .map_err(|_| HypermeshError::UnknownClassification)?;

        if point_strictly_inside_leaf(&candidate, leaf)? {
            push_unique_interior_point(
                &mut points,
                InteriorLeafPoint {
                    point: candidate,
                    planes: Some([leaf.support.clone(), first_shifted, second_shifted]),
                },
            );
        }
    }

    Ok(points)
}

fn inward_shifted_edge_plane(
    edge: &Plane,
    strict_interior_margin: &Real,
    fraction: &Real,
) -> Plane {
    let inward_offset = strict_interior_margin * fraction;
    Plane::new(edge.normal.clone(), &edge.offset - &inward_offset)
}

fn push_unique_interior_point(points: &mut Vec<InteriorLeafPoint>, point: InteriorLeafPoint) {
    if !points.iter().any(|existing| existing.point == point.point) {
        points.push(point);
    }
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
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    positive_side: bool,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut probes = Vec::new();

    if let Some(probe) =
        adjacent_normal_probe(&interior.point, support, bounds, polygons, positive_side)?
    {
        let side = classify_point(&probe, support)?;
        if side != Classification::On {
            let planes = normal_probe_planes(interior, support, &probe)?;
            probes.push(ProbePoint {
                point: probe,
                side,
                planes,
            });
        }
    }

    for axis in probe_axes(support)? {
        let normal_sign = crate::geometry::classify_real(axis_ref(&support.normal, axis))?;
        if normal_sign == Classification::On {
            continue;
        }

        let direction_positive = (normal_sign == Classification::Positive) == positive_side;
        let axis_value = axis_ref(&interior.point, axis);
        let room = if direction_positive {
            axis_ref(&bounds.max, axis) - axis_value
        } else {
            axis_value - axis_ref(&bounds.min, axis)
        };
        if !compare_real(&room, &Real::zero())?.is_gt() {
            continue;
        }

        let Some(probe) =
            adjacent_axis_probe(&interior.point, bounds, polygons, axis, direction_positive)?
        else {
            continue;
        };

        let side = classify_point(&probe, support)?;
        if side != Classification::On
            && !probes
                .iter()
                .any(|existing: &ProbePoint| existing.point == probe)
        {
            probes.push(ProbePoint {
                point: probe,
                side,
                planes: None,
            });
        }
    }

    Ok(probes)
}

fn normal_probe_planes(
    interior: &InteriorLeafPoint,
    support: &Plane,
    probe: &Point3,
) -> HypermeshResult<Option<[Plane; 3]>> {
    let Some(interior_planes) = &interior.planes else {
        return Ok(None);
    };

    let shifted_support = Plane::new(
        support.normal.clone(),
        &support.offset - &support.expression_at_point(probe),
    );
    let planes = [
        shifted_support,
        interior_planes[1].clone(),
        interior_planes[2].clone(),
    ];
    let reproduced = intersect_three_planes(&planes[0], &planes[1], &planes[2])
        .to_affine_point()
        .map_err(|_| HypermeshError::UnknownClassification)?;
    if reproduced == *probe {
        Ok(Some(planes))
    } else {
        Ok(None)
    }
}

fn adjacent_normal_probe(
    interior: &Point3,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    positive_side: bool,
) -> HypermeshResult<Option<Point3>> {
    let direction = if positive_side {
        support.normal.clone()
    } else {
        Point3::new(
            -support.normal.x.clone(),
            -support.normal.y.clone(),
            -support.normal.z.clone(),
        )
    };

    let Some(mut stop_t) = normal_probe_bounds_stop(interior, &direction, bounds)? else {
        return Ok(None);
    };

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }
        if planes_are_coplanar(&polygon.support, support)? {
            continue;
        }

        let start_value = polygon.support.expression_at_point(interior);
        if classify_real(&start_value)? == Classification::On {
            if point_lies_on_polygon(interior, polygon)? {
                return Ok(None);
            }
            continue;
        }

        let denom = dot_direction(&polygon.support.normal, &direction);
        if classify_real(&denom)? == Classification::On {
            continue;
        }
        let crossing_t =
            ((-start_value) / denom).map_err(|_| HypermeshError::UnknownClassification)?;
        if !positive_real_strictly_before(&crossing_t, &stop_t)? {
            continue;
        }

        let crossing = offset_point(interior, &direction, &crossing_t);
        if point_lies_on_polygon(&crossing, polygon)? {
            stop_t = crossing_t;
        }
    }

    if !compare_real(&stop_t, &Real::zero())?.is_gt() {
        return Ok(None);
    }
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;
    Ok(Some(offset_point(interior, &direction, &(stop_t * half))))
}

fn normal_probe_bounds_stop(
    interior: &Point3,
    direction: &Point3,
    bounds: &Aabb,
) -> HypermeshResult<Option<Real>> {
    let mut stop_t: Option<Real> = None;
    for axis in 0..3 {
        let component = axis_ref(direction, axis);
        match classify_real(component)? {
            Classification::Positive => {
                let room = axis_ref(&bounds.max, axis) - axis_ref(interior, axis);
                if !compare_real(&room, &Real::zero())?.is_gt() {
                    return Ok(None);
                }
                update_positive_stop(
                    &mut stop_t,
                    (room / component.clone())
                        .map_err(|_| HypermeshError::UnknownClassification)?,
                )?;
            }
            Classification::Negative => {
                let room = axis_ref(interior, axis) - axis_ref(&bounds.min, axis);
                if !compare_real(&room, &Real::zero())?.is_gt() {
                    return Ok(None);
                }
                update_positive_stop(
                    &mut stop_t,
                    (room / (-component.clone()))
                        .map_err(|_| HypermeshError::UnknownClassification)?,
                )?;
            }
            Classification::On => {}
        }
    }
    Ok(stop_t)
}

fn update_positive_stop(stop_t: &mut Option<Real>, candidate: Real) -> HypermeshResult<()> {
    if !compare_real(&candidate, &Real::zero())?.is_gt() {
        return Ok(());
    }
    if stop_t
        .as_ref()
        .is_none_or(|current| compare_real(&candidate, current).is_ok_and(|order| order.is_lt()))
    {
        *stop_t = Some(candidate);
    }
    Ok(())
}

fn positive_real_strictly_before(value: &Real, stop: &Real) -> HypermeshResult<bool> {
    Ok(compare_real(value, &Real::zero())?.is_gt() && compare_real(value, stop)?.is_lt())
}

fn dot_direction(left: &Point3, right: &Point3) -> Real {
    Real::signed_product_sum(
        [true, true, true],
        [
            [&left.x, &right.x],
            [&left.y, &right.y],
            [&left.z, &right.z],
        ],
    )
}

fn offset_point(point: &Point3, direction: &Point3, amount: &Real) -> Point3 {
    Point3::new(
        &point.x + &(amount * &direction.x),
        &point.y + &(amount * &direction.y),
        &point.z + &(amount * &direction.z),
    )
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::polygon::make_triangle;

    fn r(value: i32) -> Real {
        value.into()
    }

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn trace_retry_only_suppresses_unknown_classification() {
        assert_eq!(
            retryable_trace::<Vec<i32>>(Err(HypermeshError::UnknownClassification)).unwrap(),
            None
        );
        assert_eq!(
            retryable_trace::<Vec<i32>>(Err(HypermeshError::PointAtInfinity)),
            Err(HypermeshError::PointAtInfinity)
        );
    }

    #[test]
    fn trace_axis_segment_rejects_transition_dimension_mismatch() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];

        assert_eq!(
            trace_axis_segment(&p(0, 0, 0), &p(2, 0, 0), 0, &[0, 0], &[wall]),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn centroid_is_fallible_and_reports_empty_input() {
        assert_eq!(centroid(&[]).unwrap(), None);
        assert_eq!(
            centroid(&[p(0, 0, 0), p(2, 2, 2)]).unwrap(),
            Some(p(1, 1, 1))
        );
    }

    fn axis_values(points: &[Point3], axis: usize) -> Vec<Real> {
        let mut values = Vec::new();
        for point in points {
            let value = axis_ref(point, axis).clone();
            if !values.iter().any(|existing| existing == &value) {
                values.push(value);
            }
        }
        values
    }

    #[test]
    fn endpoint_box_detours_are_cut_by_surface_crossings() {
        let slanted = make_triangle(&p(0, 2, -2), &p(0, 2, 2), &p(4, -2, 0), 0, 0);

        let detours = interior_box_detour_points(&p(0, 0, 0), &p(4, 4, 4), &[slanted]).unwrap();
        let x_values = axis_values(&detours, 0);

        assert!(x_values.contains(&r(1)));
        assert!(x_values.contains(&r(3)));
    }

    #[test]
    fn shifted_edge_interior_points_move_vertices_inside_by_certified_margins() {
        let leaf = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);
        let vertices = leaf.vertices().unwrap();
        let center = centroid(&vertices).unwrap().unwrap();
        let points = shifted_edge_interior_points(&leaf, &center).unwrap();

        assert_eq!(points.len(), 3);
        for point in &points {
            assert!(point_strictly_inside_leaf(&point.point, &leaf).unwrap());
        }

        let first = &points[0].point;
        let expected_first_edge_margin =
            (leaf.edges[0].expression_at_point(&center) / Real::from(2)).unwrap();
        let expected_second_edge_margin =
            (leaf.edges[1].expression_at_point(&center) / Real::from(2)).unwrap();

        assert_eq!(
            leaf.edges[0].expression_at_point(first),
            expected_first_edge_margin
        );
        assert_eq!(
            leaf.edges[1].expression_at_point(first),
            expected_second_edge_margin
        );
    }

    #[test]
    fn bounded_probes_include_certified_normal_direction_probe() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let vertices = leaf.vertices().unwrap();
        let center = centroid(&vertices).unwrap().unwrap();
        let interior = shifted_edge_interior_points(&leaf, &center)
            .unwrap()
            .into_iter()
            .find(|point| point.planes.is_some())
            .expect("shifted edge construction should retain defining planes");

        let probes =
            bounded_probes_from_interior(&interior, &leaf.support, &bounds, true, &[]).unwrap();

        let probe = probes
            .iter()
            .find(|probe| probe.side == Classification::Positive && probe.planes.is_some())
            .expect("normal probe should preserve a shifted plane definition");
        let planes = probe.planes.as_ref().unwrap();
        assert_eq!(affine_from_planes(planes).unwrap(), probe.point);
    }

    #[test]
    fn normal_probe_is_clipped_before_intervening_surface() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let blocker = make_triangle(&p(6, 0, 0), &p(0, 6, 0), &p(0, 0, 6), 1, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));

        let probe = adjacent_normal_probe(&p(1, 1, 1), &leaf.support, &bounds, &[blocker], true)
            .unwrap()
            .expect("normal probe should stop before the blocking surface");

        assert_eq!(probe, Point3::new(q(3, 2), q(3, 2), q(3, 2)));
    }

    #[test]
    fn leaf_classification_uses_certified_slanted_normal_probe() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let ref_definitions = [axis_plane_definition(&p(0, 0, 0))];

        let winding = classify_leaf_polygon(
            &leaf.support,
            &leaf.edges,
            &p(0, 0, 0),
            &ref_definitions,
            &[0],
            &[leaf.clone()],
            &bounds,
            &leaf.delta_w,
        )
        .unwrap();

        assert_eq!(winding, vec![-1]);
    }

    #[test]
    fn plane_replacement_path_traces_certified_winding_steps() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let start = axis_plane_defined_point(&p(0, 0, 0));
        let end = axis_plane_defined_point(&p(2, 0, 0));

        let winding =
            trace_plane_replacement_path(&start.planes, &end.planes, &[0], &[wall]).unwrap();

        assert_eq!(winding, vec![-1]);
    }

    #[test]
    fn retained_reference_definitions_try_later_plane_replacement_paths() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let invalid_start = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(2)),
        ];
        let valid_start = axis_plane_defined_point(&p(0, 0, 0));
        let end = axis_plane_defined_point(&p(2, 0, 0));

        let winding = trace_probe_from_reference_definitions(
            &p(0, 0, 0),
            &[invalid_start, valid_start.planes],
            &end.planes,
            &[0],
            &[wall],
        )
        .unwrap();

        assert_eq!(winding, vec![-1]);
    }
}
