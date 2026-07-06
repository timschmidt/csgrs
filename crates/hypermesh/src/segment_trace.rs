//! Exact segment tracing for winding-number propagation.

use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};
use hyperlimit::{
    HalfspaceFeasibility, Plane3 as LimitPlane3, PredicateOutcome, classify_halfspace_feasibility3,
};

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
    planes: Vec<[Plane; 3]>,
}

#[derive(Clone, Debug, PartialEq)]
struct ProbePoint {
    point: Point3,
    side: Classification,
    planes: Vec<[Plane; 3]>,
}

#[derive(Clone, Debug, PartialEq)]
struct DetourTarget {
    point: Point3,
    definitions: Vec<[Plane; 3]>,
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

const DETOUR_RECURSION_LIMIT: usize = 2;
const PLANE_REPLACEMENT_STEP_DETOUR_LIMIT: usize = 1;

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
    trace_segment_from_definitions(
        start,
        end,
        winding,
        polygons,
        &[axis_plane_definition(start)],
        &[axis_plane_definition(end)],
    )
}

pub(crate) fn trace_segment_from_definitions(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<WindingNumberVector> {
    trace_segment_from_definitions_with_budget(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        DETOUR_RECURSION_LIMIT,
    )
}

fn trace_segment_from_definitions_with_budget(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
) -> HypermeshResult<WindingNumberVector> {
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         winding: &[i32],
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            trace_segment_with_definitions_no_detours(
                start,
                end,
                winding,
                polygons,
                start_definitions,
                end_definitions,
            )
        };
    let mut detours_for =
        |start: &Point3, end: &Point3| interior_box_detour_targets(start, end, polygons);
    trace_segment_from_definitions_with_budget_impl(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        remaining_detours,
        &mut trace_without_detours,
        &mut detours_for,
    )
}

fn trace_segment_from_definitions_with_budget_impl(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[i32],
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<WindingNumberVector> {
    if let Some(winding) =
        trace_without_detours(start, end, winding, start_definitions, end_definitions)?
    {
        return Ok(winding);
    }

    if remaining_detours == 0 {
        return Err(HypermeshError::UnknownClassification);
    }

    if let Some(winding) = trace_segment_via_detours_with_definitions_budget(
        start,
        end,
        winding,
        polygons,
        &detours_for(start, end)?,
        start_definitions,
        end_definitions,
        remaining_detours,
        trace_without_detours,
        detours_for,
    )? {
        return Ok(winding);
    }

    Err(HypermeshError::UnknownClassification)
}

fn trace_segment_without_detours(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<WindingNumberVector>> {
    if let Some(winding) = retryable_trace(trace_axis_ordered_paths(start, end, winding, polygons))?
    {
        return Ok(Some(winding));
    }

    if let Some(traced) = retryable_trace(trace_direct_segment(start, end, winding, polygons))?
        && traced.valid
    {
        return Ok(Some(traced.winding));
    }

    Ok(None)
}

fn trace_segment_via_detours_with_definitions_budget(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    detours: &[DetourTarget],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[i32],
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    for detour in detours {
        if detour.point == *start
            || detour.point == *end
            || point_lies_on_traced_surface(&detour.point, polygons)?
        {
            continue;
        }
        let Some(first_leg) = retryable_trace(trace_segment_from_definitions_with_budget_impl(
            start,
            &detour.point,
            winding,
            polygons,
            start_definitions,
            &detour.definitions,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        ))?
        else {
            continue;
        };
        let Some(second_leg) = retryable_trace(trace_segment_from_definitions_with_budget_impl(
            &detour.point,
            end,
            &first_leg,
            polygons,
            &detour.definitions,
            end_definitions,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        ))?
        else {
            continue;
        };
        return Ok(Some(second_leg));
    }

    Ok(None)
}

fn trace_segment_with_definitions_no_detours(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<Option<WindingNumberVector>> {
    if let Some(winding) = trace_segment_without_detours(start, end, winding, polygons)? {
        return Ok(Some(winding));
    }

    let mut start_definitions = start_definitions.to_vec();
    append_definition_if_missing(&mut start_definitions, axis_plane_definition(start));
    let mut end_definitions = end_definitions.to_vec();
    append_definition_if_missing(&mut end_definitions, axis_plane_definition(end));

    for start_definition in &start_definitions {
        for end_definition in &end_definitions {
            if let Some(winding) = retryable_trace(trace_plane_replacement_path_without_detours(
                start_definition,
                end_definition,
                winding,
                polygons,
            ))? {
                return Ok(Some(winding));
            }
        }
    }

    Ok(None)
}

#[cfg(test)]
fn trace_segment_with_detours_without_plane_replacement(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    remaining_detours: usize,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut trace_without_detours = |start: &Point3, end: &Point3, winding: &[i32]| {
        trace_segment_without_detours(start, end, winding, polygons)
    };
    let mut detours_for =
        |start: &Point3, end: &Point3| interior_box_detour_targets(start, end, polygons);
    trace_segment_with_detours_without_plane_replacement_impl(
        start,
        end,
        winding,
        polygons,
        remaining_detours,
        &mut trace_without_detours,
        &mut detours_for,
    )
}

#[cfg(test)]
fn trace_segment_with_detours_without_plane_replacement_impl(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[i32],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    if let Some(winding) = trace_without_detours(start, end, winding)? {
        return Ok(Some(winding));
    }

    if remaining_detours == 0 {
        return Ok(None);
    }

    for detour in detours_for(start, end)? {
        if detour.point == *start
            || detour.point == *end
            || point_lies_on_traced_surface(&detour.point, polygons)?
        {
            continue;
        }
        let Some(first_leg) = trace_segment_with_detours_without_plane_replacement_impl(
            start,
            &detour.point,
            winding,
            polygons,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        )?
        else {
            continue;
        };
        let Some(second_leg) = trace_segment_with_detours_without_plane_replacement_impl(
            &detour.point,
            end,
            &first_leg,
            polygons,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        )?
        else {
            continue;
        };
        return Ok(Some(second_leg));
    }

    Ok(None)
}

#[cfg(test)]
pub(crate) fn trace_plane_replacement_path(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_tracer(
        start_planes,
        end_planes,
        winding,
        polygons,
        |current, next, _current_planes, _next_planes, attempt, polygons| {
            retryable_trace(trace_segment(current, next, attempt, polygons))
        },
    )
}

fn trace_plane_replacement_path_without_detours(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_tracer(
        start_planes,
        end_planes,
        winding,
        polygons,
        |current, next, _current_planes, _next_planes, attempt, polygons| {
            trace_segment_without_detours(current, next, attempt, polygons)
        },
    )
}

fn trace_plane_replacement_path_with_tracer(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    mut trace_step: impl FnMut(
        &Point3,
        &Point3,
        &[Plane; 3],
        &[Plane; 3],
        &[i32],
        &[ConvexPolygon],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
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
            let mut next_planes = current_planes.clone();
            next_planes[plane_index] = end_planes[plane_index].clone();
            let next_point = match affine_from_planes(&next_planes) {
                Ok(point) => point,
                Err(HypermeshError::UnknownClassification) => {
                    valid = false;
                    break;
                }
                Err(err) => return Err(err),
            };
            if next_point != current_point {
                let Some(next_winding) = trace_step(
                    &current_point,
                    &next_point,
                    &current_planes,
                    &next_planes,
                    &attempt,
                    polygons,
                )?
                else {
                    valid = false;
                    break;
                };
                attempt = next_winding;
                current_point = next_point;
            }
            current_planes = next_planes;
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

fn interior_box_detour_targets(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<DetourTarget>> {
    let mut intervals = vec![Vec::new(), Vec::new(), Vec::new()];
    for (axis, axis_intervals) in intervals.iter_mut().enumerate() {
        let start_value = axis_ref(start, axis);
        let end_value = axis_ref(end, axis);
        if compare_real(start_value, end_value)?.is_eq() {
            axis_intervals.push((start_value.clone(), end_value.clone()));
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
            axis_intervals.push((endpoints[0].clone(), endpoints[1].clone()));
        }
    }

    let mut detours =
        Vec::with_capacity(intervals[0].len() * intervals[1].len() * intervals[2].len());
    for x in &intervals[0] {
        for y in &intervals[1] {
            for z in &intervals[2] {
                let bounds = aabb_from_axis_intervals([x, y, z])?;
                for target in strict_aabb_targets(&bounds)? {
                    push_unique_detour_target(&mut detours, target);
                }
            }
        }
    }
    Ok(detours)
}

fn aabb_from_axis_intervals(intervals: [&(Real, Real); 3]) -> HypermeshResult<Aabb> {
    let mut min = Point3::origin();
    let mut max = Point3::origin();
    for (axis, (start, end)) in intervals.into_iter().enumerate() {
        if compare_real(start, end)?.is_le() {
            *axis_mut(&mut min, axis) = start.clone();
            *axis_mut(&mut max, axis) = end.clone();
        } else {
            *axis_mut(&mut min, axis) = end.clone();
            *axis_mut(&mut max, axis) = start.clone();
        }
    }
    Ok(Aabb::new(min, max))
}

fn strict_aabb_targets(bounds: &Aabb) -> HypermeshResult<Vec<DetourTarget>> {
    let halfspaces = aabb_core_halfspaces(bounds)?;
    let report = halfspace_feasibility_report(&halfspaces)?;
    let mut targets = Vec::new();
    let report_witness = report.witness.clone();
    let seeds = strict_halfspace_cell_seeds_from_report(bounds, &halfspaces, &report)?;

    for seed in &seeds {
        let active_planes = if report_witness
            .as_ref()
            .is_some_and(|witness| witness == seed)
        {
            report.active_planes
        } else {
            [None, None, None]
        };
        push_unique_detour_target(
            &mut targets,
            DetourTarget {
                point: seed.clone(),
                definitions: probe_definitions_or_axis(
                    &seed,
                    probe_definitions_from_active_halfspaces(
                        &seed,
                        &halfspaces,
                        active_planes,
                        &[],
                    ),
                )?,
            },
        );
    }

    let mut shifted_witnesses = Vec::new();
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut shifted_witnesses,
        seeds,
        |seed| shifted_halfspace_cell_witnesses_from_seed(bounds, &halfspaces, seed),
    )?;
    for witness in shifted_witnesses {
        let point = witness.point;
        push_unique_detour_target(
            &mut targets,
            DetourTarget {
                point: point.clone(),
                definitions: probe_definitions_or_axis(
                    &point,
                    probe_definitions_from_active_halfspaces(
                        &point,
                        &witness.halfspaces,
                        witness.active_planes,
                        &[],
                    ),
                )?,
            },
        );
    }

    for witness in shifted_halfspace_cell_vertex_witnesses(bounds, &halfspaces)? {
        let point = witness.point;
        push_unique_detour_target(
            &mut targets,
            DetourTarget {
                point: point.clone(),
                definitions: probe_definitions_or_axis(
                    &point,
                    probe_definitions_from_active_halfspaces(
                        &point,
                        &witness.halfspaces,
                        witness.active_planes,
                        &[],
                    ),
                )?,
            },
        );
    }

    Ok(targets)
}

fn push_unique_detour_target(targets: &mut Vec<DetourTarget>, target: DetourTarget) {
    if let Some(existing) = targets
        .iter_mut()
        .find(|existing| existing.point == target.point)
    {
        for definition in target.definitions {
            if !existing
                .definitions
                .iter()
                .any(|candidate| candidate == &definition)
            {
                existing.definitions.push(definition);
            }
        }
    } else {
        targets.push(target);
    }
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
    search_leaf_probe_families(
        &interior_points,
        |point, positive_side| {
            bounded_probes_from_interior(point, support, bounds, positive_side, polygons)
        },
        |point, _positive_side, probe| {
            if point_lies_on_traced_surface(&probe.point, polygons)? {
                return Ok(None);
            }
            if !probe_reaches_adjacent_cell_from_interior(point, &probe, support, polygons)? {
                return Ok(None);
            }
            let Some(mut winding) =
                trace_probe_winding(ref_point, ref_definitions, &probe, ref_wnv, polygons)?
            else {
                return Ok(None);
            };
            if probe.side == Classification::Negative {
                apply_winding_transition_in_place(&mut winding, -1, host_delta_w)?;
            }
            Ok(Some(winding))
        },
    )?
    .ok_or(HypermeshError::UnknownClassification)
}

fn search_leaf_probe_families<'a>(
    interior_points: &'a [InteriorLeafPoint],
    mut probes_for: impl FnMut(&'a InteriorLeafPoint, bool) -> HypermeshResult<Vec<ProbePoint>>,
    mut handle_probe: impl FnMut(
        &'a InteriorLeafPoint,
        bool,
        ProbePoint,
    ) -> HypermeshResult<Option<WindingNumberVector>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut saw_unknown = false;

    for point in interior_points {
        for positive_side in [true, false] {
            let probes = match probes_for(point, positive_side) {
                Ok(probes) => probes,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };

            for probe in probes {
                match handle_probe(point, positive_side, probe) {
                    Ok(Some(winding)) => return Ok(Some(winding)),
                    Ok(None) => {}
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

fn trace_probe_winding(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    probe: &ProbePoint,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut probe_definitions = probe.planes.clone();
    let axis_definition = axis_plane_defined_point(&probe.point).planes;
    if !probe_definitions
        .iter()
        .any(|definition| definition == &axis_definition)
    {
        probe_definitions.push(axis_definition);
    }

    retryable_trace(
        trace_segment_from_definitions_with_step_detoured_plane_replacement(
            ref_point,
            &probe.point,
            ref_wnv,
            polygons,
            ref_definitions,
            &probe_definitions,
        ),
    )
}

pub(crate) fn trace_segment_from_definitions_with_step_detoured_plane_replacement(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<WindingNumberVector> {
    if let Some(winding) = retryable_trace(trace_segment_from_definitions(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
    ))? {
        return Ok(winding);
    }

    trace_from_definition_sets_with_step_detoured_plane_replacement(
        start,
        start_definitions,
        end,
        end_definitions,
        winding,
        polygons,
    )
}

#[cfg(test)]
fn trace_probe_from_reference_definitions(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    probe_point: &Point3,
    probe_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    trace_from_definition_sets_with_step_detoured_plane_replacement(
        ref_point,
        ref_definitions,
        probe_point,
        probe_definitions,
        ref_wnv,
        polygons,
    )
}

fn trace_from_definition_sets_with_step_detoured_plane_replacement(
    start: &Point3,
    start_definitions: &[[Plane; 3]],
    end: &Point3,
    end_definitions: &[[Plane; 3]],
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut start_definitions = start_definitions.to_vec();
    append_definition_if_missing(
        &mut start_definitions,
        axis_plane_defined_point(start).planes,
    );
    let mut end_definitions = end_definitions.to_vec();
    append_definition_if_missing(&mut end_definitions, axis_plane_defined_point(end).planes);

    for start_definition in &start_definitions {
        for end_definition in &end_definitions {
            match trace_plane_replacement_path_with_step_detours(
                start_definition,
                end_definition,
                winding,
                polygons,
            ) {
                Ok(winding) => return Ok(winding),
                Err(HypermeshError::UnknownClassification) => continue,
                Err(err) => return Err(err),
            }
        }
    }

    Err(HypermeshError::UnknownClassification)
}

fn trace_plane_replacement_path_with_step_detours(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_step_detours_impl(
        start_planes,
        end_planes,
        winding,
        polygons,
        |current, next, attempt, polygons, current_definitions, next_definitions| {
            retryable_trace(trace_segment_from_definitions(
                current,
                next,
                attempt,
                polygons,
                current_definitions,
                next_definitions,
            ))
        },
    )
}

fn trace_plane_replacement_path_with_step_detours_impl(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    mut trace_step: impl FnMut(
        &Point3,
        &Point3,
        &[i32],
        &[ConvexPolygon],
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_tracer(
        start_planes,
        end_planes,
        winding,
        polygons,
        |current, next, current_definition, next_definition, attempt, polygons| {
            trace_step(
                current,
                next,
                attempt,
                polygons,
                std::slice::from_ref(current_definition),
                std::slice::from_ref(next_definition),
            )
        },
    )
}

fn append_definition_if_missing(definitions: &mut Vec<[Plane; 3]>, candidate: [Plane; 3]) {
    if definitions
        .iter()
        .all(|definition| definition != &candidate)
    {
        definitions.push(candidate);
    }
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

fn probe_reaches_adjacent_cell_from_interior(
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut start_definitions = interior.planes.clone();
    append_definition_if_missing(
        &mut start_definitions,
        axis_plane_definition(&interior.point),
    );
    let mut end_definitions = probe.planes.clone();
    append_definition_if_missing(&mut end_definitions, axis_plane_definition(&probe.point));

    probe_reaches_adjacent_cell_with_definitions_budget(
        &interior.point,
        &probe.point,
        host_support,
        polygons,
        &start_definitions,
        &end_definitions,
        DETOUR_RECURSION_LIMIT,
    )
}

fn probe_reaches_adjacent_cell_with_definitions_budget(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
) -> HypermeshResult<bool> {
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            probe_reaches_adjacent_cell_with_definitions_no_detours(
                start,
                end,
                host_support,
                polygons,
                start_definitions,
                end_definitions,
            )
        };
    let mut detours_for =
        |start: &Point3, end: &Point3| interior_box_detour_targets(start, end, polygons);
    probe_reaches_adjacent_cell_with_definitions_budget_impl(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        remaining_detours,
        &mut trace_without_detours,
        &mut detours_for,
    )
}

fn probe_reaches_adjacent_cell_with_definitions_budget_impl(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    if trace_without_detours(start, end, start_definitions, end_definitions)? {
        return Ok(true);
    }

    if remaining_detours == 0 {
        return Ok(false);
    }

    probe_reaches_adjacent_cell_via_detours_with_budget(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        remaining_detours,
        trace_without_detours,
        detours_for,
    )
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_via_detours(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            probe_reaches_adjacent_cell_with_definitions_no_detours(
                start,
                end,
                host_support,
                polygons,
                start_definitions,
                end_definitions,
            )
        };
    let mut detours_for =
        |start: &Point3, end: &Point3| interior_box_detour_targets(start, end, polygons);
    probe_reaches_adjacent_cell_via_detours_with_budget(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        DETOUR_RECURSION_LIMIT,
        &mut trace_without_detours,
        &mut detours_for,
    )
}

fn probe_reaches_adjacent_cell_via_detours_with_budget(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for detour in detours_for(start, end)? {
        if detour.point == *start
            || detour.point == *end
            || point_lies_on_traced_surface(&detour.point, polygons)?
        {
            continue;
        }
        let first_leg = match probe_reaches_adjacent_cell_with_definitions_budget_impl(
            start,
            &detour.point,
            polygons,
            start_definitions,
            &detour.definitions,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        ) {
            Ok(result) => result,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !first_leg {
            continue;
        }
        match probe_reaches_adjacent_cell_with_definitions_budget_impl(
            &detour.point,
            end,
            polygons,
            &detour.definitions,
            end_definitions,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        ) {
            Ok(true) => return Ok(true),
            Ok(false) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

fn probe_reaches_adjacent_cell_with_definitions_no_detours(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    if probe_reaches_adjacent_cell(start, end, host_support, polygons)? {
        return Ok(true);
    }

    let mut start_definitions = start_definitions.to_vec();
    append_definition_if_missing(&mut start_definitions, axis_plane_definition(start));
    let mut end_definitions = end_definitions.to_vec();
    append_definition_if_missing(&mut end_definitions, axis_plane_definition(end));

    definition_pair_reachability_backtracking_unknown(
        &start_definitions,
        &end_definitions,
        |start_definition, end_definition| {
            plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement(
                start_definition,
                end_definition,
                host_support,
                polygons,
            )
        },
    )
}

fn probe_reaches_adjacent_cell_with_definitions_no_step_detours(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    if probe_reaches_adjacent_cell(start, end, host_support, polygons)? {
        return Ok(true);
    }

    let mut start_definitions = start_definitions.to_vec();
    append_definition_if_missing(&mut start_definitions, axis_plane_definition(start));
    let mut end_definitions = end_definitions.to_vec();
    append_definition_if_missing(&mut end_definitions, axis_plane_definition(end));

    definition_pair_reachability_backtracking_unknown(
        &start_definitions,
        &end_definitions,
        |start_definition, end_definition| {
            plane_replacement_path_reaches_adjacent_cell_without_step_detours(
                start_definition,
                end_definition,
                host_support,
                polygons,
            )
        },
    )
}

fn definition_pair_reachability_backtracking_unknown(
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    mut reaches: impl FnMut(&[Plane; 3], &[Plane; 3]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;

    for start_definition in start_definitions {
        for end_definition in end_definitions {
            match reaches(start_definition, end_definition) {
                Ok(true) => return Ok(true),
                Ok(false) => {}
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                }
                Err(err) => return Err(err),
            }
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    remaining_detours: usize,
) -> HypermeshResult<bool> {
    let mut trace_without_detours = |start: &Point3, end: &Point3| {
        probe_reaches_adjacent_cell(start, end, host_support, polygons)
    };
    let mut detours_for =
        |start: &Point3, end: &Point3| interior_box_detour_targets(start, end, polygons);
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
        start,
        end,
        polygons,
        remaining_detours,
        &mut trace_without_detours,
        &mut detours_for,
    )
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    if trace_without_detours(start, end)? {
        return Ok(true);
    }

    if remaining_detours == 0 {
        return Ok(false);
    }

    for detour in detours_for(start, end)? {
        if detour.point == *start
            || detour.point == *end
            || point_lies_on_traced_surface(&detour.point, polygons)?
        {
            continue;
        }
        if !probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
            start,
            &detour.point,
            polygons,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        )? {
            continue;
        }
        if probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
            &detour.point,
            end,
            polygons,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        )? {
            return Ok(true);
        }
    }

    Ok(false)
}

fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
) -> HypermeshResult<bool> {
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            probe_reaches_adjacent_cell_with_definitions_no_step_detours(
                start,
                end,
                host_support,
                polygons,
                start_definitions,
                end_definitions,
            )
        };
    let mut detours_for =
        |start: &Point3, end: &Point3| interior_box_detour_targets(start, end, polygons);
    probe_reaches_adjacent_cell_with_definitions_budget_impl(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        remaining_detours,
        &mut trace_without_detours,
        &mut detours_for,
    )
}

fn plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
        start_planes,
        end_planes,
        |current, next, current_definitions, next_definitions| {
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions(
                current,
                next,
                host_support,
                polygons,
                current_definitions,
                next_definitions,
                PLANE_REPLACEMENT_STEP_DETOUR_LIMIT,
            )
        },
    )
}

fn plane_replacement_path_reaches_adjacent_cell_without_step_detours(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
        start_planes,
        end_planes,
        |current, next, _current_definitions, _next_definitions| {
            probe_reaches_adjacent_cell(current, next, host_support, polygons)
        },
    )
}

fn plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    mut trace_step: impl FnMut(&Point3, &Point3, &[[Plane; 3]], &[[Plane; 3]]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for ordering in AXIS_ORDERINGS {
        let mut current_planes = start_planes.clone();
        let mut current_point = match affine_from_planes(&current_planes) {
            Ok(point) => point,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        let mut valid = true;

        for plane_index in ordering {
            let mut next_planes = current_planes.clone();
            next_planes[plane_index] = end_planes[plane_index].clone();
            let next_point = match affine_from_planes(&next_planes) {
                Ok(point) => point,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    valid = false;
                    break;
                }
                Err(err) => return Err(err),
            };
            if next_point != current_point
                && !trace_step(
                    &current_point,
                    &next_point,
                    std::slice::from_ref(&current_planes),
                    std::slice::from_ref(&next_planes),
                )?
            {
                valid = false;
                break;
            }
            current_point = next_point;
            current_planes = next_planes;
        }

        if valid {
            return Ok(true);
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
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

fn halfspace_cell_geometry_seed_candidates(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<Point3>> {
    let vertices = feasible_halfspace_cell_vertices(halfspaces)?;
    let mut candidates = Vec::new();

    for first in 0..vertices.len() {
        for second in (first + 1)..vertices.len() {
            for third in (second + 1)..vertices.len() {
                if let Some(center) = centroid(&[
                    vertices[first].clone(),
                    vertices[second].clone(),
                    vertices[third].clone(),
                ])? {
                    push_unique_halfspace_seed(&mut candidates, center);
                }
            }
        }
    }

    if let Some(center) = centroid(&vertices)? {
        push_unique_halfspace_seed(&mut candidates, center);
    }

    Ok(candidates)
}

fn interior_leaf_points(leaf: &ConvexPolygon) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let vertices = leaf.vertices()?;
    if vertices.is_empty() {
        return Ok(Vec::new());
    }

    let mut points = strict_leaf_witness_points(leaf, &vertices)?;
    let witness_points = points
        .iter()
        .map(|point| point.point.clone())
        .collect::<Vec<_>>();
    extend_interior_leaf_points_backtracking_unknown(
        &mut points,
        witness_points.iter(),
        |witness| shifted_edge_interior_points(leaf, witness),
    )?;

    Ok(points)
}

fn strict_leaf_witness_points(
    leaf: &ConvexPolygon,
    vertices: &[Point3],
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let bounds = leaf_bounds(vertices)?;
    let halfspaces = leaf_halfspaces(leaf);
    let report = halfspace_feasibility_report(&halfspaces)?;
    if report.status != HalfspaceFeasibility::Feasible {
        return Ok(Vec::new());
    }

    let report_witness = report.witness.clone();
    let mut points = Vec::new();
    let seeds = strict_leaf_witness_seeds(leaf, vertices, &bounds, &halfspaces, &report)?;

    extend_leaf_point_builds_backtracking_unknown(&mut points, seeds.iter(), |seed| {
        let active_planes = if report_witness
            .as_ref()
            .is_some_and(|witness| witness == seed)
        {
            report.active_planes
        } else {
            [None, None, None]
        };
        build_strict_leaf_point(leaf, seed, &halfspaces, active_planes)
    })?;

    let mut shifted_witnesses = Vec::new();
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut shifted_witnesses,
        seeds,
        |seed| shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, seed),
    )?;
    extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        shifted_witnesses.iter(),
        |shifted| {
            build_strict_leaf_point(
                leaf,
                &shifted.point,
                &shifted.halfspaces,
                shifted.active_planes,
            )
        },
    )?;

    let shifted_vertices = shifted_halfspace_cell_vertex_witnesses(&bounds, &halfspaces)?;
    extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        shifted_vertices.iter(),
        |shifted| {
            build_strict_leaf_point(
                leaf,
                &shifted.point,
                &shifted.halfspaces,
                shifted.active_planes,
            )
        },
    )?;

    let direct_witnesses = points
        .iter()
        .map(|point| point.point.clone())
        .collect::<Vec<_>>();
    extend_interior_leaf_points_backtracking_unknown(
        &mut points,
        direct_witnesses.iter(),
        |witness| strict_leaf_cell_points(leaf, witness),
    )?;

    Ok(points)
}

fn strict_leaf_witness_seeds(
    leaf: &ConvexPolygon,
    _vertices: &[Point3],
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: &hyperlimit::HalfspaceFeasibilityReport,
) -> HypermeshResult<Vec<Point3>> {
    let mut seeds = strict_halfspace_cell_seeds_from_report(bounds, halfspaces, report)?;

    extend_strict_halfspace_seeds_backtracking_unknown(
        &mut seeds,
        halfspace_cell_geometry_seed_candidates(halfspaces)?,
        |candidate| point_strictly_inside_leaf(candidate, leaf),
    )?;

    Ok(seeds)
}

fn leaf_bounds(vertices: &[Point3]) -> HypermeshResult<Aabb> {
    let Some(first) = vertices.first() else {
        return Err(HypermeshError::UnknownClassification);
    };

    let mut min = first.clone();
    let mut max = first.clone();
    for vertex in &vertices[1..] {
        for axis in 0..3 {
            if compare_real(axis_ref(vertex, axis), axis_ref(&min, axis))?.is_lt() {
                *axis_mut(&mut min, axis) = axis_ref(vertex, axis).clone();
            }
            if compare_real(axis_ref(vertex, axis), axis_ref(&max, axis))?.is_gt() {
                *axis_mut(&mut max, axis) = axis_ref(vertex, axis).clone();
            }
        }
    }

    Ok(Aabb::new(min, max))
}

fn leaf_halfspaces(leaf: &ConvexPolygon) -> Vec<LimitPlane3> {
    let mut halfspaces = Vec::with_capacity(leaf.edges.len() + 2);
    halfspaces.push(limit_plane_from_plane(&leaf.support));
    halfspaces.push(limit_plane_from_plane(&leaf.support.inverted()));
    for edge in &leaf.edges {
        halfspaces.push(limit_plane_from_plane(edge));
    }
    halfspaces
}

#[cfg(test)]
pub(crate) fn certified_leaf_test_point(
    support: &Plane,
    edges: &[Plane],
) -> HypermeshResult<Option<HomogeneousPoint3>> {
    let leaf = ConvexPolygon {
        support: support.clone(),
        edges: edges.to_vec(),
        mesh_index: -1,
        polygon_index: -1,
        delta_w: WindingNumberTransitionVector::new(),
        approx_bounds: None,
    };
    let points = interior_leaf_points(&leaf)?;
    let Some(point) = points
        .iter()
        .find(|point| !point.planes.is_empty())
        .or_else(|| points.first())
    else {
        return Ok(None);
    };
    Ok(Some(HomogeneousPoint3::new(
        point.point.x.clone(),
        point.point.y.clone(),
        point.point.z.clone(),
        Real::one(),
    )))
}

pub(crate) fn certified_leaf_test_points(
    support: &Plane,
    edges: &[Plane],
) -> HypermeshResult<Vec<HomogeneousPoint3>> {
    let leaf = ConvexPolygon {
        support: support.clone(),
        edges: edges.to_vec(),
        mesh_index: -1,
        polygon_index: -1,
        delta_w: WindingNumberTransitionVector::new(),
        approx_bounds: None,
    };
    Ok(interior_leaf_points(&leaf)?
        .into_iter()
        .map(|point| {
            HomogeneousPoint3::new(point.point.x, point.point.y, point.point.z, Real::one())
        })
        .collect())
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
                    planes: vec![[leaf.support.clone(), first_shifted, second_shifted]],
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
    if let Some(existing) = points
        .iter_mut()
        .find(|existing| existing.point == point.point)
    {
        for planes in point.planes {
            if !existing.planes.iter().any(|candidate| candidate == &planes) {
                existing.planes.push(planes);
            }
        }
    } else {
        points.push(point);
    }
}

fn extend_interior_leaf_points_backtracking_unknown<'a, T: 'a>(
    points: &mut Vec<InteriorLeafPoint>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<Vec<InteriorLeafPoint>>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(found) => {
                for point in found {
                    push_unique_interior_point(points, point);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if points.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

fn extend_leaf_point_builds_backtracking_unknown<'a, T: 'a>(
    points: &mut Vec<InteriorLeafPoint>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<Option<InteriorLeafPoint>>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(Some(point)) => push_unique_interior_point(points, point),
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if points.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

fn strict_leaf_cell_points(
    leaf: &ConvexPolygon,
    strict_interior: &Point3,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let vertices = leaf.vertices()?;
    let bounds = leaf_bounds(&vertices)?;
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;
    let mut halfspaces = Vec::with_capacity(leaf.edges.len() + 2);
    halfspaces.push(limit_plane_from_plane(&leaf.support));
    halfspaces.push(limit_plane_from_plane(&leaf.support.inverted()));

    for edge in &leaf.edges {
        let margin = edge.expression_at_point(strict_interior);
        if classify_real(&margin)? != Classification::Negative {
            return Ok(Vec::new());
        }
        halfspaces.push(limit_plane_from_plane(&inward_shifted_edge_plane(
            edge, &margin, &half,
        )));
    }

    let report = match classify_halfspace_feasibility3(&halfspaces) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { .. } => return Err(HypermeshError::UnknownClassification),
    };
    if report.status != HalfspaceFeasibility::Feasible {
        return Ok(Vec::new());
    }

    let mut points = Vec::new();
    let report_witness = report.witness.clone();
    let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report)?;
    extend_leaf_point_builds_backtracking_unknown(&mut points, seeds.iter(), |witness| {
        let active_planes = if report_witness
            .as_ref()
            .is_some_and(|point| point == witness)
        {
            report.active_planes
        } else {
            [None, None, None]
        };
        build_strict_leaf_point(leaf, witness, &halfspaces, active_planes)
    })?;

    let mut shifted_witnesses = Vec::new();
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut shifted_witnesses,
        seeds,
        |seed| shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, seed),
    )?;
    extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        shifted_witnesses.iter(),
        |shifted| {
            build_strict_leaf_point(
                leaf,
                &shifted.point,
                &shifted.halfspaces,
                shifted.active_planes,
            )
        },
    )?;

    let shifted_vertices = shifted_halfspace_cell_vertex_witnesses(&bounds, &halfspaces)?;
    extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        shifted_vertices.iter(),
        |shifted| {
            build_strict_leaf_point(
                leaf,
                &shifted.point,
                &shifted.halfspaces,
                shifted.active_planes,
            )
        },
    )?;

    Ok(points)
}

fn build_strict_leaf_point(
    leaf: &ConvexPolygon,
    witness: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> HypermeshResult<Option<InteriorLeafPoint>> {
    if !point_strictly_inside_leaf(witness, leaf)? {
        return Ok(None);
    }

    let planes = match leaf_interior_definitions_from_active_halfspaces(
        witness,
        &leaf.support,
        halfspaces,
        active_planes,
    ) {
        Ok(planes) => planes,
        Err(HypermeshError::UnknownClassification) => vec![axis_plane_definition(witness)],
        Err(err) => return Err(err),
    };
    Ok(Some(InteriorLeafPoint {
        point: witness.clone(),
        planes,
    }))
}

fn limit_plane_from_plane(plane: &Plane) -> LimitPlane3 {
    LimitPlane3::new(plane.normal.clone(), plane.offset.clone())
}

fn leaf_interior_definitions_from_active_halfspaces(
    witness: &Point3,
    support: &Plane,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> HypermeshResult<Vec<[Plane; 3]>> {
    let axis_definition = axis_plane_definition(witness);
    let mut definitions = Vec::new();
    let mut active = Vec::new();
    for index in active_planes.into_iter().flatten() {
        let Some(halfspace) = halfspaces.get(index) else {
            return Err(HypermeshError::UnknownClassification);
        };
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if plane == *support || plane == support.inverted() {
            continue;
        }
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if plane == *support || plane == support.inverted() {
            continue;
        }
        if !compare_real(&plane.expression_at_point(witness), &Real::zero())?.is_eq() {
            continue;
        }
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    if active.len() >= 2 {
        for first in 0..active.len() {
            for second in (first + 1)..active.len() {
                push_verified_leaf_definition(
                    &mut definitions,
                    [
                        support.clone(),
                        active[first].clone(),
                        active[second].clone(),
                    ],
                    witness,
                )?;
            }
        }
    }

    for plane in &active {
        for axis in axis_definition.iter().cloned() {
            push_verified_leaf_definition(
                &mut definitions,
                [support.clone(), plane.clone(), axis],
                witness,
            )?;
        }
    }

    for first_axis in 0..3 {
        for second_axis in (first_axis + 1)..3 {
            push_verified_leaf_definition(
                &mut definitions,
                [
                    support.clone(),
                    axis_definition[first_axis].clone(),
                    axis_definition[second_axis].clone(),
                ],
                witness,
            )?;
        }
    }

    if definitions.is_empty() {
        return Err(HypermeshError::UnknownClassification);
    }
    Ok(definitions)
}

fn push_verified_leaf_definition(
    definitions: &mut Vec<[Plane; 3]>,
    definition: [Plane; 3],
    witness: &Point3,
) -> HypermeshResult<()> {
    match intersect_three_planes(&definition[0], &definition[1], &definition[2]).to_affine_point() {
        Ok(point) if point == *witness => {
            if !definitions.iter().any(|existing| existing == &definition) {
                definitions.push(definition);
            }
        }
        Ok(_) | Err(_) => {}
    }
    Ok(())
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
    let mut saw_unknown = false;

    extend_probe_families_backtracking_unknown(
        &mut probes,
        adjacent_normal_probes(interior, support, bounds, polygons, positive_side),
        &mut saw_unknown,
    )?;

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

        extend_probe_families_backtracking_unknown(
            &mut probes,
            adjacent_axis_probes(
                interior,
                support,
                bounds,
                polygons,
                axis,
                direction_positive,
            ),
            &mut saw_unknown,
        )?;
    }

    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(probes)
    }
}

fn extend_probe_families_backtracking_unknown(
    probes: &mut Vec<ProbePoint>,
    family: HypermeshResult<Vec<ProbePoint>>,
    saw_unknown: &mut bool,
) -> HypermeshResult<()> {
    match family {
        Ok(found) => {
            for probe in found {
                push_unique_probe_point(probes, probe);
            }
            Ok(())
        }
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            Ok(())
        }
        Err(err) => Err(err),
    }
}

fn probe_definitions_from_active_halfspaces(
    witness: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    extra_planes: &[Plane],
) -> HypermeshResult<Vec<[Plane; 3]>> {
    let axis_definition = axis_plane_definition(witness);
    let mut definitions = Vec::new();
    let mut active = Vec::new();

    for plane in extra_planes {
        if !active.iter().any(|existing| existing == plane) {
            active.push(plane.clone());
        }
    }

    for index in active_planes.into_iter().flatten() {
        let Some(halfspace) = halfspaces.get(index) else {
            return Err(HypermeshError::UnknownClassification);
        };
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if !compare_real(&plane.expression_at_point(witness), &Real::zero())?.is_eq() {
            continue;
        }
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    for first in 0..active.len() {
        for second in (first + 1)..active.len() {
            for third in (second + 1)..active.len() {
                push_verified_probe_definition(
                    &mut definitions,
                    [
                        active[first].clone(),
                        active[second].clone(),
                        active[third].clone(),
                    ],
                    witness,
                )?;
            }
        }
    }

    for first in 0..active.len() {
        for second in (first + 1)..active.len() {
            for axis in 0..3 {
                push_verified_probe_definition(
                    &mut definitions,
                    [
                        active[first].clone(),
                        active[second].clone(),
                        axis_definition[axis].clone(),
                    ],
                    witness,
                )?;
            }
        }
    }

    for plane in &active {
        for first_axis in 0..3 {
            for second_axis in (first_axis + 1)..3 {
                push_verified_probe_definition(
                    &mut definitions,
                    [
                        plane.clone(),
                        axis_definition[first_axis].clone(),
                        axis_definition[second_axis].clone(),
                    ],
                    witness,
                )?;
            }
        }
    }

    push_verified_probe_definition(&mut definitions, axis_definition, witness)?;
    Ok(definitions)
}

fn probe_definitions_or_axis(
    witness: &Point3,
    result: HypermeshResult<Vec<[Plane; 3]>>,
) -> HypermeshResult<Vec<[Plane; 3]>> {
    match result {
        Ok(planes) => Ok(planes),
        Err(HypermeshError::UnknownClassification) => Ok(vec![axis_plane_definition(witness)]),
        Err(err) => Err(err),
    }
}

fn push_verified_probe_definition(
    definitions: &mut Vec<[Plane; 3]>,
    definition: [Plane; 3],
    witness: &Point3,
) -> HypermeshResult<()> {
    match affine_from_planes(&definition) {
        Ok(point) if point == *witness => {
            if !definitions.iter().any(|existing| existing == &definition) {
                definitions.push(definition);
            }
        }
        Ok(_) | Err(HypermeshError::UnknownClassification) => {}
        Err(err) => return Err(err),
    }
    Ok(())
}

fn adjacent_normal_probes(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    positive_side: bool,
) -> HypermeshResult<Vec<ProbePoint>> {
    let direction = if positive_side {
        support.normal.clone()
    } else {
        Point3::new(
            -support.normal.x.clone(),
            -support.normal.y.clone(),
            -support.normal.z.clone(),
        )
    };

    let Some(mut stop_t) = normal_probe_bounds_stop(&interior.point, &direction, bounds)? else {
        return Ok(Vec::new());
    };

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }
        if planes_are_coplanar(&polygon.support, support)? {
            continue;
        }

        let start_value = polygon.support.expression_at_point(&interior.point);
        if classify_real(&start_value)? == Classification::On {
            if point_lies_on_polygon(&interior.point, polygon)? {
                return Ok(Vec::new());
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

        let crossing = offset_point(&interior.point, &direction, &crossing_t);
        if point_lies_on_polygon(&crossing, polygon)? {
            stop_t = crossing_t;
        }
    }

    if !compare_real(&stop_t, &Real::zero())?.is_gt() {
        return Ok(Vec::new());
    }
    let stop_point = offset_point(&interior.point, &direction, &stop_t);
    let corridor = bounds_between_points(&interior.point, &stop_point)?;
    collect_normal_probe_targets(&interior.planes, |definition| {
        if let Some(definition) = definition
            && !normal_probe_definition_preserves_support_direction(definition, support)?
        {
            return Ok(Vec::new());
        }
        strict_normal_probe_targets(
            interior,
            support,
            &corridor,
            definition,
            &stop_point,
            positive_side,
        )
    })
}

fn push_unique_probe_point(probes: &mut Vec<ProbePoint>, probe: ProbePoint) {
    if let Some(existing) = probes
        .iter_mut()
        .find(|existing| existing.point == probe.point && existing.side == probe.side)
    {
        for definition in probe.planes {
            if !existing
                .planes
                .iter()
                .any(|candidate| candidate == &definition)
            {
                existing.planes.push(definition);
            }
        }
    } else {
        probes.push(probe);
    }
}

fn collect_normal_probe_targets(
    definitions: &[[Plane; 3]],
    mut search: impl FnMut(Option<&[Plane; 3]>) -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut probes = Vec::new();
    let mut saw_unknown = false;
    for definition in definitions {
        match search(Some(definition)) {
            Ok(found) => {
                for probe in found {
                    push_unique_probe_point(&mut probes, probe);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    match search(None) {
        Ok(found) => {
            for probe in found {
                push_unique_probe_point(&mut probes, probe);
            }
        }
        Err(HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }
    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(probes)
    }
}

fn normal_probe_definition_preserves_support_direction(
    definition: &[Plane; 3],
    support: &Plane,
) -> HypermeshResult<bool> {
    Ok(
        classify_real(&dot_direction(&definition[1].normal, &support.normal))?
            == Classification::On
            && classify_real(&dot_direction(&definition[2].normal, &support.normal))?
                == Classification::On,
    )
}

fn strict_normal_probe_targets(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
    positive_side: bool,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let report = halfspace_feasibility_report(&halfspaces)?;
    let mut probes = Vec::new();
    let mut extra_planes = Vec::new();
    let seeds = strict_halfspace_cell_seeds_from_report(corridor, &halfspaces, &report)?;
    for definition in &interior.planes {
        for plane in &definition[1..] {
            if !extra_planes.iter().any(|existing| existing == plane) {
                extra_planes.push(plane.clone());
            }
        }
    }

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_probe_point(
            witness,
            support,
            &halfspaces,
            report.active_planes,
            &extra_planes,
        )
    })?;

    let mut shifted_witnesses = Vec::new();
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut shifted_witnesses,
        seeds,
        |seed| shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, seed),
    )?;
    extend_probe_point_builds_backtracking_unknown(
        &mut probes,
        shifted_witnesses.iter(),
        |shifted| {
            build_probe_point(
                &shifted.point,
                support,
                &shifted.halfspaces,
                shifted.active_planes,
                &extra_planes,
            )
        },
    )?;

    let shifted_vertices = shifted_halfspace_cell_vertex_witnesses(corridor, &halfspaces)?;
    extend_probe_point_builds_backtracking_unknown(
        &mut probes,
        shifted_vertices.iter(),
        |witness| {
            build_probe_point(
                &witness.point,
                support,
                &witness.halfspaces,
                witness.active_planes,
                &extra_planes,
            )
        },
    )?;

    Ok(probes)
}

fn bounds_between_points(start: &Point3, end: &Point3) -> HypermeshResult<Aabb> {
    let mut min = Point3::origin();
    let mut max = Point3::origin();
    for axis in 0..3 {
        let start_value = axis_ref(start, axis);
        let end_value = axis_ref(end, axis);
        if compare_real(start_value, end_value)?.is_le() {
            *axis_mut(&mut min, axis) = start_value.clone();
            *axis_mut(&mut max, axis) = end_value.clone();
        } else {
            *axis_mut(&mut min, axis) = end_value.clone();
            *axis_mut(&mut max, axis) = start_value.clone();
        }
    }
    Ok(Aabb::new(min, max))
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

fn adjacent_axis_probes(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
) -> HypermeshResult<Vec<ProbePoint>> {
    let start_value = axis_ref(&interior.point, axis);
    let bound_value = if direction_positive {
        axis_ref(&bounds.max, axis)
    } else {
        axis_ref(&bounds.min, axis)
    };
    if !axis_value_after_start(start_value, bound_value, direction_positive)? {
        return Ok(Vec::new());
    }

    let mut endpoint = interior.point.clone();
    *axis_mut(&mut endpoint, axis) = bound_value.clone();
    let mut stop_value = bound_value.clone();

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let Some(crossing) = segment_plane_crossing(&interior.point, &endpoint, &polygon.support)?
        else {
            continue;
        };
        if !point_strictly_between_axis(&crossing, &interior.point, &endpoint, axis)? {
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
        return Ok(Vec::new());
    }

    let corridor = axis_probe_bounds(&interior.point, axis, &stop_value)?;
    collect_axis_probe_targets(&interior.planes, |definition| {
        if let Some(definition) = definition
            && !axis_probe_definition_preserves_axis_direction(definition, axis)?
        {
            return Ok(Vec::new());
        }
        strict_axis_probe_targets(
            interior,
            support,
            &corridor,
            axis,
            direction_positive,
            definition,
        )
    })
}

fn axis_probe_bounds(interior: &Point3, axis: usize, stop_value: &Real) -> HypermeshResult<Aabb> {
    let mut min = interior.clone();
    let mut max = interior.clone();
    let start_value = axis_ref(interior, axis);
    if compare_real(start_value, stop_value)?.is_lt() {
        *axis_mut(&mut max, axis) = stop_value.clone();
    } else {
        *axis_mut(&mut min, axis) = stop_value.clone();
    }
    Ok(Aabb::new(min, max))
}

fn collect_axis_probe_targets(
    definitions: &[[Plane; 3]],
    mut search: impl FnMut(Option<&[Plane; 3]>) -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut probes = Vec::new();
    let mut saw_unknown = false;
    for definition in definitions {
        match search(Some(definition)) {
            Ok(found) => {
                for probe in found {
                    push_unique_probe_point(&mut probes, probe);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    match search(None) {
        Ok(found) => {
            for probe in found {
                push_unique_probe_point(&mut probes, probe);
            }
        }
        Err(HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }
    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(probes)
    }
}

fn extend_probe_point_builds_backtracking_unknown<'a, T: 'a>(
    probes: &mut Vec<ProbePoint>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<Option<ProbePoint>>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(Some(probe)) => push_unique_probe_point(probes, probe),
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

fn axis_probe_definition_preserves_axis_direction(
    definition: &[Plane; 3],
    axis: usize,
) -> HypermeshResult<bool> {
    Ok(
        classify_real(axis_ref(&definition[1].normal, axis))? == Classification::On
            && classify_real(axis_ref(&definition[2].normal, axis))? == Classification::On,
    )
}

fn strict_axis_probe_targets(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    axis: usize,
    positive_side: bool,
    definition: Option<&[Plane; 3]>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    let report = halfspace_feasibility_report(&halfspaces)?;
    let mut probes = Vec::new();
    let seeds = strict_halfspace_cell_seeds_from_report(corridor, &halfspaces, &report)?;

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_axis_probe_point(
            witness,
            interior,
            support,
            axis,
            definition,
            &halfspaces,
            report.active_planes,
        )
    })?;

    let mut shifted_witnesses = Vec::new();
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut shifted_witnesses,
        seeds,
        |seed| shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, seed),
    )?;
    extend_probe_point_builds_backtracking_unknown(
        &mut probes,
        shifted_witnesses.iter(),
        |shifted| {
            build_axis_probe_point(
                &shifted.point,
                interior,
                support,
                axis,
                definition,
                &shifted.halfspaces,
                shifted.active_planes,
            )
        },
    )?;

    let shifted_vertices = shifted_halfspace_cell_vertex_witnesses(corridor, &halfspaces)?;
    extend_probe_point_builds_backtracking_unknown(
        &mut probes,
        shifted_vertices.iter(),
        |witness| {
            build_axis_probe_point(
                &witness.point,
                interior,
                support,
                axis,
                definition,
                &witness.halfspaces,
                witness.active_planes,
            )
        },
    )?;

    Ok(probes)
}

fn build_probe_point(
    witness: &Point3,
    support: &Plane,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    extra_planes: &[Plane],
) -> HypermeshResult<Option<ProbePoint>> {
    if !point_satisfies_halfspaces(witness, halfspaces)? {
        return Ok(None);
    }
    let side = classify_point(witness, support)?;
    if side == Classification::On {
        return Ok(None);
    }

    let shifted_support = Plane::new(
        support.normal.clone(),
        &support.offset - &support.expression_at_point(witness),
    );
    let mut all_extra_planes = vec![shifted_support];
    for plane in extra_planes {
        if !all_extra_planes.iter().any(|existing| existing == plane) {
            all_extra_planes.push(plane.clone());
        }
    }

    Ok(Some(ProbePoint {
        point: witness.clone(),
        side,
        planes: probe_definitions_or_axis(
            witness,
            probe_definitions_from_active_halfspaces(
                witness,
                halfspaces,
                active_planes,
                &all_extra_planes,
            ),
        )?,
    }))
}

fn build_axis_probe_point(
    witness: &Point3,
    interior: &InteriorLeafPoint,
    support: &Plane,
    axis: usize,
    definition: Option<&[Plane; 3]>,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> HypermeshResult<Option<ProbePoint>> {
    if !point_satisfies_halfspaces(witness, halfspaces)? {
        return Ok(None);
    }
    let side = classify_point(witness, support)?;
    if side == Classification::On {
        return Ok(None);
    }

    Ok(Some(ProbePoint {
        point: witness.clone(),
        side,
        planes: probe_definitions_or_axis(
            witness,
            axis_probe_definitions(
                interior,
                support,
                axis,
                definition,
                halfspaces,
                active_planes,
                witness,
            ),
        )?,
    }))
}

fn axis_probe_definitions(
    interior: &InteriorLeafPoint,
    support: &Plane,
    axis: usize,
    definition: Option<&[Plane; 3]>,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    witness: &Point3,
) -> HypermeshResult<Vec<[Plane; 3]>> {
    let shifted_support = Plane::new(
        support.normal.clone(),
        &support.offset - &support.expression_at_point(witness),
    );
    let axes = other_axes(axis);
    let mut extra_planes = vec![
        shifted_support,
        Plane::axis_aligned(axes[0], axis_ref(&interior.point, axes[0]).clone()),
        Plane::axis_aligned(axes[1], axis_ref(&interior.point, axes[1]).clone()),
    ];
    if let Some(definition) = definition {
        for plane in &definition[1..] {
            if !extra_planes.iter().any(|existing| existing == plane) {
                extra_planes.push(plane.clone());
            }
        }
    }
    probe_definitions_from_active_halfspaces(witness, halfspaces, active_planes, &extra_planes)
}

fn aabb_core_halfspaces(bounds: &Aabb) -> HypermeshResult<Vec<LimitPlane3>> {
    let mut halfspaces = Vec::with_capacity(6);
    for axis in 0..3 {
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        halfspaces.push(axis_halfspace(axis, true, min.clone()));
        halfspaces.push(axis_halfspace(axis, false, max.clone()));
    }
    Ok(halfspaces)
}

fn axis_halfspace(axis: usize, lower_bound: bool, value: Real) -> LimitPlane3 {
    let zero = Real::zero();
    let one = Real::one();
    let minus_one = -Real::one();
    let normal = match (axis, lower_bound) {
        (0, true) => Point3::new(minus_one, zero.clone(), zero),
        (1, true) => Point3::new(zero.clone(), minus_one, zero),
        (2, true) => Point3::new(zero.clone(), zero, minus_one),
        (0, false) => Point3::new(one, zero.clone(), zero),
        (1, false) => Point3::new(zero.clone(), one, zero),
        (2, false) => Point3::new(zero.clone(), zero, one),
        _ => panic!("axis must be in 0..3"),
    };
    let offset = if lower_bound { value } else { -value };
    LimitPlane3::new(normal, offset)
}

fn plane_halfspace(plane: &Plane) -> LimitPlane3 {
    LimitPlane3::new(plane.normal.clone(), plane.offset.clone())
}

fn negated_halfspace(halfspace: &LimitPlane3) -> LimitPlane3 {
    LimitPlane3::new(
        Point3::new(
            -halfspace.normal.x.clone(),
            -halfspace.normal.y.clone(),
            -halfspace.normal.z.clone(),
        ),
        -halfspace.offset.clone(),
    )
}

fn push_plane_equality_halfspaces(halfspaces: &mut Vec<LimitPlane3>, plane: &Plane) {
    let halfspace = plane_halfspace(plane);
    halfspaces.push(halfspace.clone());
    halfspaces.push(negated_halfspace(&halfspace));
}

fn support_side_halfspace(plane: &Plane, positive: bool) -> LimitPlane3 {
    if positive {
        LimitPlane3::new(
            Point3::new(
                -plane.normal.x.clone(),
                -plane.normal.y.clone(),
                -plane.normal.z.clone(),
            ),
            -plane.offset.clone(),
        )
    } else {
        LimitPlane3::new(plane.normal.clone(), plane.offset.clone())
    }
}

fn normal_stop_halfspace(plane: &Plane, stop_point: &Point3, positive_side: bool) -> LimitPlane3 {
    let stop_plane = Plane::new(
        plane.normal.clone(),
        &plane.offset - &plane.expression_at_point(stop_point),
    );
    let halfspace = plane_halfspace(&stop_plane);
    if positive_side {
        halfspace
    } else {
        negated_halfspace(&halfspace)
    }
}

fn strict_halfspace_cell_seeds_from_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: &hyperlimit::HalfspaceFeasibilityReport,
) -> HypermeshResult<Vec<Point3>> {
    let mut seeds = Vec::new();

    if report.status == HalfspaceFeasibility::Feasible
        && let Some(witness) = &report.witness
    {
        extend_strict_halfspace_seeds_backtracking_unknown(
            &mut seeds,
            std::iter::once(witness.clone()),
            |candidate| point_strictly_inside_halfspace_cell(candidate, bounds, halfspaces),
        )?;
    }

    extend_strict_halfspace_seeds_backtracking_unknown(
        &mut seeds,
        feasible_halfspace_cell_vertices(halfspaces)?,
        |candidate| point_strictly_inside_halfspace_cell(candidate, bounds, halfspaces),
    )?;

    Ok(seeds)
}

fn push_unique_halfspace_seed(seeds: &mut Vec<Point3>, seed: Point3) {
    if !seeds.iter().any(|existing| existing == &seed) {
        seeds.push(seed);
    }
}

fn extend_strict_halfspace_seeds_backtracking_unknown(
    seeds: &mut Vec<Point3>,
    candidates: impl IntoIterator<Item = Point3>,
    mut is_strict_seed: impl FnMut(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match is_strict_seed(&candidate) {
            Ok(true) => push_unique_halfspace_seed(seeds, candidate),
            Ok(false) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if seeds.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

struct ShiftedHalfspaceWitness {
    point: Point3,
    halfspaces: Vec<LimitPlane3>,
    active_planes: [Option<usize>; 3],
}

fn shifted_halfspace_cell_witnesses_from_seed(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>> {
    let shifted = shifted_halfspace_cell(bounds, halfspaces, seed)?;
    let shifted_report = halfspace_feasibility_report(&shifted)?;
    if shifted_report.status != HalfspaceFeasibility::Feasible {
        return Ok(Vec::new());
    }

    let report_witness = shifted_report.witness.clone();
    let mut witnesses = Vec::new();
    for witness in strict_halfspace_cell_seeds_from_report(bounds, &shifted, &shifted_report)? {
        let active_planes = if report_witness
            .as_ref()
            .is_some_and(|point| point == &witness)
        {
            shifted_report.active_planes
        } else {
            [None, None, None]
        };
        push_unique_shifted_halfspace_witness(
            &mut witnesses,
            ShiftedHalfspaceWitness {
                point: witness,
                halfspaces: shifted.clone(),
                active_planes,
            },
        );
    }
    for witness in feasible_halfspace_cell_vertices(&shifted)? {
        if !point_strictly_inside_halfspace_cell(&witness, bounds, halfspaces)? {
            continue;
        }
        push_unique_shifted_halfspace_witness(
            &mut witnesses,
            ShiftedHalfspaceWitness {
                point: witness,
                halfspaces: shifted.clone(),
                active_planes: [None, None, None],
            },
        );
    }

    Ok(witnesses)
}

fn shifted_halfspace_cell_vertex_witnesses(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>> {
    let mut witnesses: Vec<ShiftedHalfspaceWitness> = Vec::new();
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        feasible_halfspace_cell_vertices(halfspaces)?,
        |seed| shifted_halfspace_cell_witnesses_from_seed(bounds, halfspaces, seed),
    )?;
    Ok(witnesses)
}

fn push_unique_shifted_halfspace_witness(
    witnesses: &mut Vec<ShiftedHalfspaceWitness>,
    witness: ShiftedHalfspaceWitness,
) {
    if !witnesses
        .iter()
        .any(|existing| existing.point == witness.point)
    {
        witnesses.push(witness);
    }
}

fn extend_shifted_halfspace_witnesses_backtracking_unknown(
    witnesses: &mut Vec<ShiftedHalfspaceWitness>,
    seeds: impl IntoIterator<Item = Point3>,
    mut build: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for seed in seeds {
        match build(&seed) {
            Ok(found) => {
                for witness in found {
                    push_unique_shifted_halfspace_witness(witnesses, witness);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if witnesses.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

fn halfspace_feasibility_report(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<hyperlimit::HalfspaceFeasibilityReport> {
    match classify_halfspace_feasibility3(halfspaces) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { .. } => Err(HypermeshError::UnknownClassification),
    }
}

fn feasible_halfspace_cell_vertices(halfspaces: &[LimitPlane3]) -> HypermeshResult<Vec<Point3>> {
    let mut vertices = Vec::new();
    for first in 0..halfspaces.len() {
        for second in (first + 1)..halfspaces.len() {
            for third in (second + 1)..halfspaces.len() {
                let candidate = intersect_three_planes(
                    &halfspaces[first],
                    &halfspaces[second],
                    &halfspaces[third],
                );
                let Ok(point) = candidate.to_affine_point() else {
                    continue;
                };
                if point_satisfies_halfspaces(&point, halfspaces)?
                    && !vertices.iter().any(|existing| existing == &point)
                {
                    vertices.push(point);
                }
            }
        }
    }
    Ok(vertices)
}

fn point_satisfies_halfspaces(point: &Point3, halfspaces: &[LimitPlane3]) -> HypermeshResult<bool> {
    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if classify_point(point, &plane)? == Classification::Positive {
            return Ok(false);
        }
    }
    Ok(true)
}

fn point_strictly_inside_halfspace_cell(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    if !point_strictly_inside_probe_bounds(point, bounds)? {
        return Ok(false);
    }
    for halfspace in halfspaces {
        if halfspace_is_degenerate_bound(halfspace, bounds)?
            || halfspace_has_opposite_pair(halfspace, halfspaces)
        {
            continue;
        }
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if compare_real(&plane.expression_at_point(point), &Real::zero())?.is_eq() {
            return Ok(false);
        }
    }
    Ok(true)
}

fn point_strictly_inside_probe_bounds(point: &Point3, bounds: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        if compare_real(min, max)?.is_eq() {
            if compare_real(axis_ref(point, axis), min)?.is_ne() {
                return Ok(false);
            }
            continue;
        }
        if !compare_real(axis_ref(point, axis), min)?.is_gt()
            || !compare_real(axis_ref(point, axis), max)?.is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn shifted_halfspace_cell(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    strict_interior: &Point3,
) -> HypermeshResult<Vec<LimitPlane3>> {
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;
    let mut shifted = Vec::with_capacity(halfspaces.len());
    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        let value = plane.expression_at_point(strict_interior);
        let keep_closed = compare_real(&value, &Real::zero())?.is_eq()
            || halfspace_is_degenerate_bound(halfspace, bounds)?;
        let offset = if keep_closed {
            halfspace.offset.clone()
        } else {
            &halfspace.offset - &(value * &half)
        };
        shifted.push(LimitPlane3::new(halfspace.normal.clone(), offset));
    }
    Ok(shifted)
}

fn halfspace_is_degenerate_bound(halfspace: &LimitPlane3, bounds: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        if compare_real(min, max)?.is_ne() {
            continue;
        }
        if *halfspace == axis_halfspace(axis, true, min.clone())
            || *halfspace == axis_halfspace(axis, false, min.clone())
        {
            return Ok(true);
        }
    }
    Ok(false)
}

fn halfspace_has_opposite_pair(target: &LimitPlane3, halfspaces: &[LimitPlane3]) -> bool {
    let opposite = negated_halfspace(target);
    halfspaces.iter().any(|halfspace| halfspace == &opposite)
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
    use crate::polygon::{make_quad, make_triangle};

    fn r(value: i32) -> Real {
        value.into()
    }

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    fn px(x: Real, y: i32, z: i32) -> Point3 {
        Point3::new(x, r(y), r(z))
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

        let detours = interior_box_detour_targets(&p(0, 0, 0), &p(4, 4, 4), &[slanted]).unwrap();
        let x_values = axis_values(
            &detours
                .iter()
                .map(|target| target.point.clone())
                .collect::<Vec<_>>(),
            0,
        );

        assert!(
            x_values
                .iter()
                .any(|value| compare_real(value, &r(0)).unwrap().is_gt()
                    && compare_real(value, &r(2)).unwrap().is_lt())
        );
        assert!(
            x_values
                .iter()
                .any(|value| compare_real(value, &r(2)).unwrap().is_gt()
                    && compare_real(value, &r(4)).unwrap().is_lt())
        );
    }

    #[test]
    fn strict_aabb_targets_handle_degenerate_axis_boxes() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 6, 0));
        let targets = strict_aabb_targets(&bounds).unwrap();

        assert!(!targets.is_empty());
        for target in targets {
            assert_eq!(target.point.z, r(0));
            assert!(compare_real(&target.point.x, &r(0)).unwrap().is_gt());
            assert!(compare_real(&target.point.x, &r(4)).unwrap().is_lt());
            assert!(compare_real(&target.point.y, &r(0)).unwrap().is_gt());
            assert!(compare_real(&target.point.y, &r(6)).unwrap().is_lt());
            assert!(!target.definitions.is_empty());
        }
    }

    #[test]
    fn shifted_halfspace_cell_vertex_witnesses_return_strict_points() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let witnesses = shifted_halfspace_cell_vertex_witnesses(&bounds, &halfspaces).unwrap();

        assert!(!witnesses.is_empty());
        for witness in &witnesses {
            assert!(
                point_strictly_inside_halfspace_cell(&witness.point, &bounds, &halfspaces).unwrap()
            );
        }
    }

    #[test]
    fn shifted_halfspace_cell_witnesses_from_seed_returns_only_strict_points() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let witnesses =
            shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, &p(2, 1, 3)).unwrap();

        assert!(!witnesses.is_empty());
        for witness in &witnesses {
            assert!(
                point_strictly_inside_halfspace_cell(&witness.point, &bounds, &halfspaces).unwrap()
            );
        }
    }

    #[test]
    fn shifted_halfspace_cell_witnesses_from_seed_include_shifted_vertex_targets() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let witnesses =
            shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, &p(2, 1, 3)).unwrap();

        assert!(
            witnesses
                .iter()
                .any(|witness| witness.point == Point3::new(r(3), q(5, 2), q(7, 2)))
        );
        assert!(
            witnesses
                .iter()
                .find(|witness| witness.point == Point3::new(r(3), q(5, 2), q(7, 2)))
                .is_some_and(|witness| witness.active_planes == [None, None, None])
        );
    }

    #[test]
    fn shifted_halfspace_witness_collection_backtracks_after_uncertified_seed() {
        let first_seed = p(1, 1, 1);
        let second_seed = p(2, 2, 2);
        let kept = ShiftedHalfspaceWitness {
            point: p(3, 3, 3),
            halfspaces: Vec::new(),
            active_planes: [None, None, None],
        };
        let mut witnesses = Vec::new();

        extend_shifted_halfspace_witnesses_backtracking_unknown(
            &mut witnesses,
            vec![first_seed.clone(), second_seed.clone()],
            |seed| {
                if seed == &first_seed {
                    Err(HypermeshError::UnknownClassification)
                } else if seed == &second_seed {
                    Ok(vec![ShiftedHalfspaceWitness {
                        point: kept.point.clone(),
                        halfspaces: kept.halfspaces.clone(),
                        active_planes: kept.active_planes,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(witnesses.len(), 1);
        assert_eq!(witnesses[0].point, kept.point);
    }

    #[test]
    fn shifted_halfspace_witness_collection_reports_unknown_if_all_seeds_are_uncertified() {
        let first_seed = p(1, 1, 1);
        let second_seed = p(2, 2, 2);
        let mut witnesses = Vec::new();

        let err = extend_shifted_halfspace_witnesses_backtracking_unknown(
            &mut witnesses,
            vec![first_seed, second_seed],
            |_seed| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn strict_halfspace_cell_seeds_include_direct_strict_feasibility_witness() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let direct = p(2, 1, 3);
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(direct.clone(), [None, None, None]);

        let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        assert!(seeds.iter().any(|seed| seed == &direct));
    }

    #[test]
    fn strict_halfspace_cell_seeds_include_strict_feasible_vertices() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = vec![
            axis_halfspace(0, true, r(1)),
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, true, r(2)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, true, r(3)),
            axis_halfspace(2, false, r(3)),
        ];
        let report = hyperlimit::HalfspaceFeasibilityReport::feasible(
            Point3::new(r(1), r(2), r(0)),
            [None, None, None],
        );

        let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        assert_eq!(seeds, vec![Point3::new(r(1), r(2), r(3))]);
    }

    #[test]
    fn strict_halfspace_cell_seed_collection_backtracks_after_uncertified_candidate() {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut seeds = Vec::new();

        extend_strict_halfspace_seeds_backtracking_unknown(
            &mut seeds,
            vec![first.clone(), second.clone()],
            |candidate| {
                if candidate == &first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(candidate == &second)
                }
            },
        )
        .unwrap();

        assert_eq!(seeds, vec![second]);
    }

    #[test]
    fn strict_halfspace_cell_seed_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut seeds = Vec::new();

        let err = extend_strict_halfspace_seeds_backtracking_unknown(
            &mut seeds,
            vec![first, second],
            |_candidate| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn strict_leaf_witness_seeds_include_strict_halfspace_triangle_centroid() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let vertices = leaf.vertices().unwrap();
        let bounds = leaf_bounds(&vertices).unwrap();
        let halfspaces = leaf_halfspaces(&leaf);
        let report = halfspace_feasibility_report(&halfspaces).unwrap();
        let center = centroid(&feasible_halfspace_cell_vertices(&halfspaces).unwrap())
            .unwrap()
            .unwrap();

        let seeds =
            strict_leaf_witness_seeds(&leaf, &vertices, &bounds, &halfspaces, &report).unwrap();

        assert!(point_strictly_inside_leaf(&center, &leaf).unwrap());
        assert!(seeds.iter().any(|seed| seed == &center));
    }

    #[test]
    fn strict_leaf_witness_seeds_include_strict_halfspace_geometry_family() {
        let leaf = make_quad(&p(0, 0, 0), &p(4, 0, 0), &p(4, 4, 0), &p(0, 4, 0), 0, 0);
        let vertices = leaf.vertices().unwrap();
        let bounds = leaf_bounds(&vertices).unwrap();
        let halfspaces = leaf_halfspaces(&leaf);
        let report = halfspace_feasibility_report(&halfspaces).unwrap();
        let triangle_center = centroid(&[p(0, 0, 0), p(4, 0, 0), p(4, 4, 0)])
            .unwrap()
            .unwrap();

        let seeds =
            strict_leaf_witness_seeds(&leaf, &vertices, &bounds, &halfspaces, &report).unwrap();

        assert!(point_strictly_inside_leaf(&triangle_center, &leaf).unwrap());
        assert!(seeds.iter().any(|seed| seed == &triangle_center));
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
            .find(|point| !point.planes.is_empty())
            .expect("shifted edge construction should retain defining planes");

        let probes =
            bounded_probes_from_interior(&interior, &leaf.support, &bounds, true, &[]).unwrap();

        let probe = probes
            .iter()
            .find(|probe| probe.side == Classification::Positive && !probe.planes.is_empty())
            .expect("normal probe should preserve a shifted plane definition");
        let planes = &probe.planes[0];
        assert_eq!(affine_from_planes(planes).unwrap(), probe.point);
    }

    #[test]
    fn bounded_probe_family_collection_backtracks_after_uncertified_family() {
        let constrained_probe = ProbePoint {
            point: p(1, 1, 1),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
        };
        let mut probes = Vec::new();
        let mut saw_unknown = false;

        extend_probe_families_backtracking_unknown(
            &mut probes,
            Err(HypermeshError::UnknownClassification),
            &mut saw_unknown,
        )
        .unwrap();
        extend_probe_families_backtracking_unknown(
            &mut probes,
            Ok(vec![constrained_probe.clone()]),
            &mut saw_unknown,
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, constrained_probe.point);
    }

    #[test]
    fn bounded_probe_family_collection_reports_unknown_if_all_families_are_uncertified() {
        let mut probes = Vec::new();
        let mut saw_unknown = false;

        extend_probe_families_backtracking_unknown(
            &mut probes,
            Err(HypermeshError::UnknownClassification),
            &mut saw_unknown,
        )
        .unwrap();
        extend_probe_families_backtracking_unknown(
            &mut probes,
            Err(HypermeshError::UnknownClassification),
            &mut saw_unknown,
        )
        .unwrap();

        assert!(saw_unknown);
        assert!(probes.is_empty());
    }

    #[test]
    fn leaf_probe_family_search_backtracks_after_uncertified_probe_family() {
        let first = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
        };
        let second = InteriorLeafPoint {
            point: p(2, 2, 2),
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
        };
        let winning_probe = ProbePoint {
            point: p(3, 3, 3),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(3, 3, 3))],
        };

        let winding = search_leaf_probe_families(
            &[first.clone(), second.clone()],
            |point, _positive_side| {
                if point.point == first.point {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(vec![winning_probe.clone()])
                }
            },
            |point, _positive_side, _probe| {
                if point.point == second.point {
                    Ok(Some(vec![1]))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert_eq!(winding, Some(vec![1]));
    }

    #[test]
    fn leaf_probe_family_search_backtracks_after_uncertified_probe_check() {
        let first = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
        };
        let second = InteriorLeafPoint {
            point: p(2, 2, 2),
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
        };
        let probe = ProbePoint {
            point: p(3, 3, 3),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(3, 3, 3))],
        };

        let winding = search_leaf_probe_families(
            &[first.clone(), second.clone()],
            |_point, _positive_side| Ok(vec![probe.clone()]),
            |point, _positive_side, _probe| {
                if point.point == first.point {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(vec![2]))
                }
            },
        )
        .unwrap();

        assert_eq!(winding, Some(vec![2]));
    }

    #[test]
    fn leaf_probe_family_search_reports_unknown_if_all_families_are_uncertified() {
        let point = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
        };

        let err = search_leaf_probe_families(
            &[point],
            |_point, _positive_side| Err(HypermeshError::UnknownClassification),
            |_point, _positive_side, _probe| Ok(Some(vec![1])),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn normal_probe_is_clipped_before_intervening_surface() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let blocker = make_triangle(&p(6, 0, 0), &p(0, 6, 0), &p(0, 0, 6), 1, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let vertices = leaf.vertices().unwrap();
        let center = centroid(&vertices).unwrap().unwrap();
        let interior = shifted_edge_interior_points(&leaf, &center)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("shifted edge construction should retain defining planes");

        let probes =
            adjacent_normal_probes(&interior, &leaf.support, &bounds, &[blocker.clone()], true)
                .unwrap();
        let probe = probes
            .into_iter()
            .find(|probe| probe.side == Classification::Positive)
            .expect("normal corridor should contain a certified probe witness");

        assert!(!probe.planes.is_empty());
        for definition in &probe.planes {
            assert_eq!(affine_from_planes(definition).unwrap(), probe.point);
        }
        let start_value = leaf.support.expression_at_point(&interior.point);
        let probe_value = leaf.support.expression_at_point(&probe.point);
        let blocker_value = blocker.support.expression_at_point(&probe.point);
        assert!(compare_real(&probe_value, &start_value).unwrap().is_gt());
        assert!(compare_real(&blocker_value, &Real::zero()).unwrap().is_lt());
    }

    #[test]
    fn collect_normal_probe_targets_keeps_unrestricted_family_after_definition_hits() {
        let support = Plane::axis_aligned(2, r(0));
        let definition = [
            support.clone(),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let constrained_probe = ProbePoint {
            point: p(1, 1, 1),
            side: Classification::Positive,
            planes: vec![definition.clone()],
        };
        let unrestricted_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
        };

        let probes = collect_normal_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Ok(vec![constrained_probe.clone()]),
            None => Ok(vec![unrestricted_probe.clone()]),
        })
        .unwrap();

        assert_eq!(probes.len(), 2);
        assert!(
            probes
                .iter()
                .any(|probe| probe.point == constrained_probe.point)
        );
        assert!(
            probes
                .iter()
                .any(|probe| probe.point == unrestricted_probe.point)
        );
    }

    #[test]
    fn collect_normal_probe_targets_merges_duplicate_unrestricted_probe_definitions() {
        let support = Plane::axis_aligned(2, r(0));
        let definition_probe = ProbePoint {
            point: p(1, 1, 1),
            side: Classification::Positive,
            planes: vec![[
                support.clone(),
                Plane::axis_aligned(0, r(1)),
                Plane::axis_aligned(1, r(1)),
            ]],
        };
        let extra_definition = [
            support,
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(2, r(1)),
        ];

        let probes =
            collect_normal_probe_targets(&[definition_probe.planes[0].clone()], |candidate| {
                match candidate {
                    Some(_) => Ok(vec![definition_probe.clone()]),
                    None => Ok(vec![ProbePoint {
                        point: definition_probe.point.clone(),
                        side: definition_probe.side,
                        planes: vec![extra_definition.clone()],
                    }]),
                }
            })
            .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].planes.len(), 2);
    }

    #[test]
    fn collect_normal_probe_targets_backtracks_after_uncertified_definition() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let unrestricted_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
        };

        let probes = collect_normal_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Err(HypermeshError::UnknownClassification),
            None => Ok(vec![unrestricted_probe.clone()]),
        })
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, unrestricted_probe.point);
    }

    #[test]
    fn collect_normal_probe_targets_report_unknown_if_all_families_are_uncertified() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];

        let err = collect_normal_probe_targets(&[definition], |_candidate| {
            Err(HypermeshError::UnknownClassification)
        })
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_point_build_collection_backtracks_after_uncertified_candidate() {
        let mut probes = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_probe_point_builds_backtracking_unknown(
            &mut probes,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                if *candidate == first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(ProbePoint {
                        point: candidate.clone(),
                        side: Classification::Positive,
                        planes: vec![axis_plane_definition(candidate)],
                    }))
                }
            },
        )
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, second);
    }

    #[test]
    fn probe_point_build_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let mut probes = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        let err = extend_probe_point_builds_backtracking_unknown(
            &mut probes,
            [first, second].iter(),
            |_candidate| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn adjacent_axis_probe_uses_corridor_witness_and_retains_definition() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
        };

        let probe = adjacent_axis_probes(&interior, &leaf.support, &bounds, &[], 0, true)
            .unwrap()
            .into_iter()
            .find(|probe| probe.side == Classification::Positive)
            .expect("axis corridor should contain a certified probe witness");

        assert_eq!(probe.side, Classification::Positive);
        assert!(!probe.planes.is_empty());
        for definition in &probe.planes {
            assert_eq!(affine_from_planes(definition).unwrap(), probe.point);
        }
        assert!(compare_real(&probe.point.x, &r(1)).unwrap().is_gt());
        assert!(compare_real(&probe.point.x, &r(4)).unwrap().is_lt());
        assert_eq!(probe.point.y, r(1));
        assert_eq!(probe.point.z, r(1));
    }

    #[test]
    fn collect_axis_probe_targets_backtracks_after_uncertified_definition() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let unrestricted_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
        };

        let probes = collect_axis_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Err(HypermeshError::UnknownClassification),
            None => Ok(vec![unrestricted_probe.clone()]),
        })
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, unrestricted_probe.point);
    }

    #[test]
    fn collect_axis_probe_targets_report_unknown_if_all_families_are_uncertified() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];

        let err = collect_axis_probe_targets(&[definition], |_candidate| {
            Err(HypermeshError::UnknownClassification)
        })
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn collect_axis_probe_targets_keeps_unrestricted_family_after_definition_hits() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let constrained_probe = ProbePoint {
            point: p(1, 1, 1),
            side: Classification::Positive,
            planes: vec![definition.clone()],
        };
        let unrestricted_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
        };

        let probes = collect_axis_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Ok(vec![constrained_probe.clone()]),
            None => Ok(vec![unrestricted_probe.clone()]),
        })
        .unwrap();

        assert_eq!(probes.len(), 2);
        assert!(
            probes
                .iter()
                .any(|probe| probe.point == constrained_probe.point)
        );
        assert!(
            probes
                .iter()
                .any(|probe| probe.point == unrestricted_probe.point)
        );
    }

    #[test]
    fn adjacent_axis_probe_preserves_retained_definition_when_axis_direction_allows() {
        let support = Plane::axis_aligned(2, r(0));
        let bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
        let retained = [
            support.clone(),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![retained.clone()],
        };

        let probe = adjacent_axis_probes(&interior, &support, &bounds, &[], 2, true)
            .unwrap()
            .into_iter()
            .find(|probe| {
                probe.side == Classification::Positive
                    && probe
                        .planes
                        .iter()
                        .any(|planes| planes[1] == retained[1] && planes[2] == retained[2])
            })
            .expect("axis-direction probe should preserve retained axis-stable planes");

        assert_eq!(probe.point.x, r(1));
        assert_eq!(probe.point.y, r(1));
        assert!(compare_real(&probe.point.z, &r(0)).unwrap().is_gt());
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
    fn strict_leaf_cell_points_retain_replayable_planes() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let center = p(1, 1, 1);

        let interior = strict_leaf_cell_points(&leaf, &center)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("strict leaf halfspaces should have a feasible witness");

        assert!(point_strictly_inside_leaf(&interior.point, &leaf).unwrap());
        assert!(!interior.planes.is_empty());
        let planes = &interior.planes[0];
        assert_eq!(affine_from_planes(planes).unwrap(), interior.point);
        assert_eq!(planes[0], leaf.support);
    }

    #[test]
    fn strict_leaf_cell_points_include_shifted_leaf_vertices() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let center = p(1, 1, 1);
        let vertices = leaf.vertices().unwrap();
        let bounds = leaf_bounds(&vertices).unwrap();
        let half = (Real::one() / Real::from(2)).unwrap();
        let mut halfspaces = vec![
            limit_plane_from_plane(&leaf.support),
            limit_plane_from_plane(&leaf.support.inverted()),
        ];
        for edge in &leaf.edges {
            let margin = edge.expression_at_point(&center);
            halfspaces.push(limit_plane_from_plane(&inward_shifted_edge_plane(
                edge, &margin, &half,
            )));
        }

        let report = halfspace_feasibility_report(&halfspaces).unwrap();
        let report_witness = report.witness.clone();
        let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();
        let mut direct_points = Vec::new();
        for seed in seeds {
            let active_planes = if report_witness.as_ref().is_some_and(|point| point == &seed) {
                report.active_planes
            } else {
                [None, None, None]
            };
            if let Some(point) =
                build_strict_leaf_point(&leaf, &seed, &halfspaces, active_planes).unwrap()
            {
                direct_points.push(point.point);
            }
        }

        let interiors = strict_leaf_cell_points(&leaf, &center).unwrap();
        let shifted = interiors
            .iter()
            .find(|point| !direct_points.iter().any(|direct| direct == &point.point))
            .expect("shifted strict leaf witness family should extend direct seed points");

        assert!(!shifted.planes.is_empty());
    }

    #[test]
    fn strict_leaf_cell_points_return_only_strict_points() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let center = p(1, 1, 1);

        let interiors = strict_leaf_cell_points(&leaf, &center).unwrap();

        assert!(!interiors.is_empty());
        for interior in &interiors {
            assert!(point_strictly_inside_leaf(&interior.point, &leaf).unwrap());
        }
    }

    #[test]
    fn strict_leaf_witness_points_include_shifted_leaf_vertices() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let vertices = leaf.vertices().unwrap();

        let interiors = strict_leaf_witness_points(&leaf, &vertices).unwrap();

        assert!(
            interiors
                .iter()
                .any(|point| point.point == Point3::new(q(1, 2), q(1, 2), r(2)))
        );
        assert!(interiors.iter().all(|point| !point.planes.is_empty()));
    }

    #[test]
    fn strict_leaf_witness_points_extend_direct_family_with_stricter_leaf_cells() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let vertices = leaf.vertices().unwrap();
        let bounds = leaf_bounds(&vertices).unwrap();
        let halfspaces = leaf_halfspaces(&leaf);
        let report = halfspace_feasibility_report(&halfspaces).unwrap();
        let report_witness = report.witness.clone();
        let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        let mut direct_points = Vec::new();
        for seed in &seeds {
            let active_planes = if report_witness.as_ref().is_some_and(|point| point == seed) {
                report.active_planes
            } else {
                [None, None, None]
            };
            if let Some(point) =
                build_strict_leaf_point(&leaf, seed, &halfspaces, active_planes).unwrap()
            {
                direct_points.push(point.point);
            }
        }

        let mut stricter_points = Vec::new();
        for point in &direct_points {
            for stricter in strict_leaf_cell_points(&leaf, point).unwrap() {
                if !direct_points.iter().any(|direct| direct == &stricter.point) {
                    stricter_points.push(stricter.point);
                }
            }
        }

        let interiors = strict_leaf_witness_points(&leaf, &vertices).unwrap();

        assert!(!stricter_points.is_empty());
        assert!(
            stricter_points
                .iter()
                .any(|point| interiors.iter().any(|interior| &interior.point == point))
        );
    }

    #[test]
    fn interior_leaf_point_collection_backtracks_after_uncertified_candidate() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_interior_leaf_points_backtracking_unknown(
            &mut points,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                if *candidate == first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(vec![InteriorLeafPoint {
                        point: candidate.clone(),
                        planes: vec![axis_plane_definition(candidate)],
                    }])
                }
            },
        )
        .unwrap();

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].point, second);
    }

    #[test]
    fn interior_leaf_point_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        let err = extend_interior_leaf_points_backtracking_unknown(
            &mut points,
            [first, second].iter(),
            |_candidate| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn leaf_point_build_collection_backtracks_after_uncertified_candidate() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_leaf_point_builds_backtracking_unknown(
            &mut points,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                if *candidate == first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(InteriorLeafPoint {
                        point: candidate.clone(),
                        planes: vec![axis_plane_definition(candidate)],
                    }))
                }
            },
        )
        .unwrap();

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].point, second);
    }

    #[test]
    fn leaf_point_build_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        let err = extend_leaf_point_builds_backtracking_unknown(
            &mut points,
            [first, second].iter(),
            |_candidate| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn certified_leaf_test_point_prefers_replayable_interior_witness() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let expected_points = interior_leaf_points(&leaf)
            .unwrap()
            .into_iter()
            .filter(|point| !point.planes.is_empty())
            .map(|point| point.point)
            .collect::<Vec<_>>();

        let point = certified_leaf_test_point(&leaf.support, &leaf.edges)
            .unwrap()
            .expect("triangle leaf should have a certified strict interior point")
            .to_affine_point()
            .unwrap();

        assert!(!expected_points.is_empty());
        assert!(expected_points.iter().any(|expected| expected == &point));
    }

    #[test]
    fn interior_leaf_points_drop_naked_centroid_when_replayable_points_exist() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);

        let points = interior_leaf_points(&leaf).unwrap();

        assert!(!points.is_empty());
        assert!(points.iter().all(|point| !point.planes.is_empty()));
    }

    #[test]
    fn leaf_interior_definitions_include_non_basis_active_halfspaces() {
        let witness = p(1, 1, 1);
        let support = Plane::axis_aligned(2, r(1));
        let halfspaces = vec![
            limit_plane_from_plane(&support),
            limit_plane_from_plane(&support.inverted()),
            LimitPlane3::new(p(1, 0, 0), r(-1)),
            LimitPlane3::new(p(0, 1, 0), r(-1)),
            LimitPlane3::new(p(1, 1, 1), r(-3)),
        ];

        let definitions = leaf_interior_definitions_from_active_halfspaces(
            &witness,
            &support,
            &halfspaces,
            [Some(0), Some(2), Some(3)],
        )
        .unwrap();

        assert!(definitions.iter().any(|definition| {
            definition[1..]
                .iter()
                .any(|plane| plane.normal == p(1, 1, 1))
        }));
        for definition in &definitions {
            assert_eq!(definition[0], support);
            assert_eq!(affine_from_planes(definition).unwrap(), witness);
        }
    }

    #[test]
    fn strict_leaf_witness_retains_axis_definition_when_active_replay_fails() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let witness = p(1, 1, 1);
        let halfspaces = vec![limit_plane_from_plane(&leaf.support)];

        let point = build_strict_leaf_point(&leaf, &witness, &halfspaces, [Some(9), None, None])
            .unwrap()
            .expect("strict witness should still be retained");

        assert_eq!(point.point, witness);
        assert_eq!(point.planes, vec![axis_plane_definition(&point.point)]);
    }

    #[test]
    fn probe_definitions_include_non_basis_active_halfspaces() {
        let witness = p(1, 1, 1);
        let shifted_support = Plane::axis_aligned(2, r(1));
        let halfspaces = vec![
            LimitPlane3::new(p(1, 0, 0), r(-1)),
            LimitPlane3::new(p(0, 1, 0), r(-1)),
            LimitPlane3::new(p(1, 1, 1), r(-3)),
        ];

        let definitions = probe_definitions_from_active_halfspaces(
            &witness,
            &halfspaces,
            [Some(0), Some(1), None],
            &[shifted_support],
        )
        .unwrap();

        assert!(
            definitions
                .iter()
                .any(|definition| { definition.iter().any(|plane| plane.normal == p(1, 1, 1)) })
        );
        for definition in &definitions {
            assert_eq!(affine_from_planes(definition).unwrap(), witness);
        }
    }

    #[test]
    fn probe_definitions_or_axis_falls_back_to_axis_definition() {
        let witness = p(1, 2, 3);

        let definitions =
            probe_definitions_or_axis(&witness, Err(HypermeshError::UnknownClassification))
                .unwrap();

        assert_eq!(definitions, vec![axis_plane_definition(&witness)]);
    }

    #[test]
    fn strict_probe_witness_retains_axis_definition_when_active_replay_fails() {
        let support = Plane::axis_aligned(2, r(0));
        let witness = p(1, 1, 1);
        let halfspaces = vec![axis_halfspace(2, false, r(1))];

        let probe = build_probe_point(&witness, &support, &halfspaces, [Some(9), None, None], &[])
            .unwrap()
            .expect("strict probe witness should still be retained");

        assert_eq!(probe.point, witness);
        assert_eq!(probe.planes, vec![axis_plane_definition(&probe.point)]);
    }

    #[test]
    fn strict_axis_probe_witness_retains_axis_definition_when_active_replay_fails() {
        let support = Plane::axis_aligned(2, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![axis_plane_definition(&p(1, 1, 0))],
        };
        let witness = p(2, 1, 1);
        let halfspaces = vec![axis_halfspace(0, false, r(2))];

        let probe = build_axis_probe_point(
            &witness,
            &interior,
            &support,
            0,
            None,
            &halfspaces,
            [Some(9), None, None],
        )
        .unwrap()
        .expect("strict axis probe witness should still be retained");

        assert_eq!(probe.point, witness);
        assert_eq!(probe.planes, vec![axis_plane_definition(&probe.point)]);
    }

    #[test]
    fn duplicate_probe_points_merge_plane_definitions() {
        let point = p(1, 1, 1);
        let first_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let second_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(2, r(1)),
        ];
        let mut probes = vec![ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: vec![first_definition.clone()],
        }];

        push_unique_probe_point(
            &mut probes,
            ProbePoint {
                point,
                side: Classification::Positive,
                planes: vec![second_definition.clone()],
            },
        );
        push_unique_probe_point(
            &mut probes,
            ProbePoint {
                point: p(1, 1, 1),
                side: Classification::Positive,
                planes: vec![second_definition],
            },
        );

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].planes.len(), 2);
    }

    #[test]
    fn duplicate_interior_points_merge_plane_definitions() {
        let point = p(1, 1, 1);
        let mut points = vec![InteriorLeafPoint {
            point: point.clone(),
            planes: Vec::new(),
        }];
        let first_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let second_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(2, r(1)),
        ];

        push_unique_interior_point(
            &mut points,
            InteriorLeafPoint {
                point: point.clone(),
                planes: vec![first_definition.clone()],
            },
        );
        push_unique_interior_point(
            &mut points,
            InteriorLeafPoint {
                point,
                planes: vec![second_definition.clone()],
            },
        );

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].planes, vec![first_definition, second_definition]);
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
            &p(2, 0, 0),
            std::slice::from_ref(&end.planes),
            &[0],
            &[wall],
        )
        .unwrap();

        assert_eq!(winding, vec![-1]);
    }

    #[test]
    fn retained_probe_definitions_try_later_plane_replacement_paths() {
        let ref_point = p(0, 0, 0);
        let ref_definitions = [[
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ]];
        let invalid_probe_definition = [
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(0)),
        ];
        let probe = ProbePoint {
            point: p(2, 1, 0),
            side: Classification::Positive,
            planes: vec![invalid_probe_definition, axis_plane_definition(&p(2, 1, 0))],
        };
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];

        assert_eq!(
            trace_segment(&ref_point, &probe.point, &[0], &[wall.clone()]),
            Err(HypermeshError::UnknownClassification)
        );

        let winding =
            trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[wall]).unwrap();

        assert_eq!(winding, Some(vec![0]));
    }

    #[test]
    fn detour_legs_retry_direct_paths_when_axis_order_fails() {
        let blockers = vec![
            make_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(1, 1, 0), 0, 0),
            make_triangle(&p(0, 1, 0), &p(1, 1, 0), &p(0, 2, 0), 0, 1),
            make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 0, 2),
        ];

        assert_eq!(
            trace_axis_ordered_paths(&p(0, 0, 0), &p(1, 1, 1), &[0], &blockers),
            Err(HypermeshError::UnknownClassification)
        );
        assert_eq!(
            trace_direct_segment(&p(0, 0, 0), &p(1, 1, 1), &[0], &blockers)
                .unwrap()
                .winding,
            vec![0]
        );

        let traced = trace_segment_via_detours_with_definitions_budget(
            &p(0, 0, 0),
            &p(2, 2, 2),
            &[0],
            &blockers,
            &[DetourTarget {
                point: p(1, 1, 1),
                definitions: vec![axis_plane_definition(&p(1, 1, 1))],
            }],
            &[axis_plane_definition(&p(0, 0, 0))],
            &[axis_plane_definition(&p(2, 2, 2))],
            1,
            &mut |start, end, winding, start_definitions, end_definitions| {
                trace_segment_with_definitions_no_detours(
                    start,
                    end,
                    winding,
                    &blockers,
                    start_definitions,
                    end_definitions,
                )
            },
            &mut |_start, _end| Ok(Vec::new()),
        )
        .unwrap();

        assert_eq!(traced, Some(vec![0]));
    }

    #[test]
    fn detour_legs_retry_plane_replacement_from_detour_definitions() {
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];
        let detour = DetourTarget {
            point: p(2, 1, 0),
            definitions: vec![[
                Plane::axis_aligned(0, r(2)),
                Plane::axis_aligned(1, r(1)),
                Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            ]],
        };

        assert_eq!(
            trace_segment_without_detours(&p(0, 0, 0), &detour.point, &[0], &[wall.clone()])
                .unwrap(),
            None
        );

        let traced = trace_segment_via_detours_with_definitions_budget(
            &p(0, 0, 0),
            &p(2, 2, 0),
            &[0],
            &[wall.clone()],
            &[detour],
            &[axis_plane_definition(&p(0, 0, 0))],
            &[axis_plane_definition(&p(2, 2, 0))],
            1,
            &mut |start, end, winding, start_definitions, end_definitions| {
                trace_segment_with_definitions_no_detours(
                    start,
                    end,
                    winding,
                    &[wall.clone()],
                    start_definitions,
                    end_definitions,
                )
            },
            &mut |_start, _end| Ok(Vec::new()),
        )
        .unwrap();

        assert_eq!(traced, Some(vec![0]));
    }

    #[test]
    fn detour_legs_can_use_retained_start_definitions() {
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];
        let start = p(0, 0, 0);
        let end = p(2, 2, 0);
        let start_definitions = [[
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ]];
        let detour = DetourTarget {
            point: p(2, 1, 0),
            definitions: vec![axis_plane_definition(&p(2, 1, 0))],
        };

        let without_retained_start = trace_segment_via_detours_with_definitions_budget(
            &start,
            &end,
            &[0],
            &[wall.clone()],
            std::slice::from_ref(&detour),
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut |start, end, winding, start_definitions, end_definitions| {
                trace_segment_with_definitions_no_detours(
                    start,
                    end,
                    winding,
                    &[wall.clone()],
                    start_definitions,
                    end_definitions,
                )
            },
            &mut |_start, _end| Ok(Vec::new()),
        )
        .unwrap();
        assert_eq!(without_retained_start, None);

        let with_retained_start = trace_segment_via_detours_with_definitions_budget(
            &start,
            &end,
            &[0],
            &[wall.clone()],
            &[detour],
            &start_definitions,
            &[axis_plane_definition(&end)],
            1,
            &mut |start, end, winding, start_definitions, end_definitions| {
                trace_segment_with_definitions_no_detours(
                    start,
                    end,
                    winding,
                    &[wall.clone()],
                    start_definitions,
                    end_definitions,
                )
            },
            &mut |_start, _end| Ok(Vec::new()),
        )
        .unwrap();

        assert_eq!(with_retained_start, Some(vec![0]));
    }

    #[test]
    fn axis_defined_probes_retry_plane_replacement_from_reference_definitions() {
        let ref_point = p(0, 0, 0);
        let ref_definitions = [[
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ]];
        let probe = ProbePoint {
            point: p(2, 1, 0),
            side: Classification::Positive,
            planes: Vec::new(),
        };
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];

        assert_eq!(
            trace_segment(&ref_point, &probe.point, &[0], &[wall.clone()]),
            Err(HypermeshError::UnknownClassification)
        );

        let winding =
            trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[wall]).unwrap();

        assert_eq!(winding, Some(vec![0]));
    }

    #[test]
    fn probe_reachability_retries_plane_replacement_from_retained_definitions() {
        let host_support = Plane::axis_aligned(2, r(0));
        let blocker = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![[
                Plane::axis_aligned(0, r(0)),
                Plane::axis_aligned(1, r(0)),
                Plane::from_coefficients(r(1), r(1), r(1), r(0)),
            ]],
        };
        let probe = ProbePoint {
            point: p(2, 1, 1),
            side: Classification::Positive,
            planes: vec![[
                Plane::axis_aligned(0, r(2)),
                Plane::axis_aligned(1, r(1)),
                Plane::from_coefficients(r(1), r(1), r(1), r(-4)),
            ]],
        };

        assert!(
            !probe_reaches_adjacent_cell(
                &interior.point,
                &probe.point,
                &host_support,
                std::slice::from_ref(&blocker),
            )
            .unwrap()
        );
        assert!(probe_reaches_adjacent_cell_from_interior(
            &interior,
            &probe,
            &host_support,
            &[blocker],
        )
        .unwrap());
    }

    #[test]
    fn probe_step_detour_helper_retries_lower_definition_trace() {
        let host_support = Plane::axis_aligned(2, r(0));
        let blocker = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![[
                Plane::axis_aligned(0, r(0)),
                Plane::axis_aligned(1, r(0)),
                Plane::from_coefficients(r(1), r(1), r(1), r(0)),
            ]],
        };
        let probe = ProbePoint {
            point: p(2, 1, 1),
            side: Classification::Positive,
            planes: vec![[
                Plane::axis_aligned(0, r(2)),
                Plane::axis_aligned(1, r(1)),
                Plane::from_coefficients(r(1), r(1), r(1), r(-4)),
            ]],
        };

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions(
                &interior.point,
                &probe.point,
                &host_support,
                &[blocker],
                &interior.planes,
                &probe.planes,
                0,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_surfaces_unknown_when_arrangement_detour_needs_uncertified_replacement_leg() {
        let host_support = Plane::axis_aligned(2, r(0));
        let start = p(0, 0, 0);
        let end = p(4, 4, 4);
        let mut blockers = vec![
            make_triangle(&p(4, 0, 0), &p(5, 0, 0), &p(4, 1, 0), 0, 0),
            make_triangle(&p(0, 4, 0), &p(1, 4, 0), &p(0, 5, 0), 0, 1),
            make_triangle(&p(0, 0, 4), &p(1, 0, 4), &p(0, 1, 4), 0, 2),
        ];

        for (index, x) in [q(4, 3), r(2), q(8, 3)].into_iter().enumerate() {
            blockers.push(make_triangle(
                &px(x.clone(), -1, -1),
                &px(x.clone(), 5, -1),
                &px(x, 2, 5),
                0,
                3 + index as isize,
            ));
        }

        assert!(!probe_reaches_adjacent_cell(&start, &end, &host_support, &blockers).unwrap());
        assert_eq!(
            probe_reaches_adjacent_cell_via_detours(
                &start,
                &end,
                &host_support,
                &blockers,
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
            )
            .unwrap_err(),
            HypermeshError::UnknownClassification
        );
    }

    #[test]
    fn recursive_probe_reachability_budget_retries_detour_legs() {
        let start = p(0, 0, 0);
        let inner = p(1, 0, 0);
        let outer = p(2, 0, 0);
        let end = p(3, 0, 0);
        let outer_target = DetourTarget {
            point: outer.clone(),
            definitions: vec![axis_plane_definition(&outer)],
        };
        let inner_target = DetourTarget {
            point: inner.clone(),
            definitions: vec![axis_plane_definition(&inner)],
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                Ok((*from == start && *to == inner)
                    || (*from == inner && *to == outer)
                    || (*from == outer && *to == end))
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![outer_target.clone()])
            } else if *from == start && *to == outer {
                Ok(vec![inner_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            !probe_reaches_adjacent_cell_with_definitions_budget_impl(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );

        assert!(
            probe_reaches_adjacent_cell_with_definitions_budget_impl(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                2,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_backtracks_after_uncertified_detour_leg() {
        let start = p(0, 0, 0);
        let blocked = p(1, 0, 0);
        let good = p(2, 0, 0);
        let end = p(3, 0, 0);
        let blocked_target = DetourTarget {
            point: blocked.clone(),
            definitions: vec![axis_plane_definition(&blocked)],
        };
        let good_target = DetourTarget {
            point: good.clone(),
            definitions: vec![axis_plane_definition(&good)],
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                if *from == start && *to == blocked {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok((*from == start && *to == good) || (*from == good && *to == end))
                }
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![blocked_target.clone(), good_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            probe_reaches_adjacent_cell_via_detours_with_budget(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_reports_unknown_if_all_detours_are_uncertified() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour.clone(),
            definitions: vec![axis_plane_definition(&detour)],
        };
        let mut trace_without_detours =
            |_from: &Point3,
             _to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                Err(HypermeshError::UnknownClassification)
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert_eq!(
            probe_reaches_adjacent_cell_via_detours_with_budget(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            ),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn probe_plane_replacement_step_detour_budget_uses_single_detour() {
        let start = p(0, 0, 0);
        let detour_point = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour_point.clone(),
            definitions: vec![axis_plane_definition(&detour_point)],
        };
        let mut trace_without_detours = |from: &Point3, to: &Point3| {
            Ok((*from == start && *to == detour_point) || (*from == detour_point && *to == end))
        };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            !probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
                &start,
                &end,
                &[],
                0,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
                &start,
                &end,
                &[],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_winding_plane_replacement_step_detour_budget_uses_single_detour() {
        let start = p(0, 0, 0);
        let detour_point = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour_point.clone(),
            definitions: vec![axis_plane_definition(&detour_point)],
        };
        let mut trace_without_detours = |from: &Point3, to: &Point3, winding: &[i32]| {
            if (*from == start && *to == detour_point) || (*from == detour_point && *to == end) {
                Ok(Some(winding.to_vec()))
            } else {
                Ok(None)
            }
        };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert_eq!(
            trace_segment_with_detours_without_plane_replacement_impl(
                &start,
                &end,
                &[0],
                &[],
                0,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap(),
            None
        );

        assert_eq!(
            trace_segment_with_detours_without_plane_replacement_impl(
                &start,
                &end,
                &[0],
                &[],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap(),
            Some(vec![0])
        );
    }

    #[test]
    fn probe_plane_replacement_step_detours_preserve_intermediate_definitions() {
        let start = p(0, 0, 0);
        let detour_point = p(1, 0, 0);
        let end = p(2, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let end_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-2)),
            Plane::axis_aligned(1, r(0)),
            Plane::axis_aligned(2, r(0)),
        ];
        let detour_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-1)),
            Plane::axis_aligned(1, r(0)),
            Plane::axis_aligned(2, r(0)),
        ];
        let detour_target = DetourTarget {
            point: detour_point.clone(),
            definitions: vec![detour_definition.clone()],
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             start_definitions: &[[Plane; 3]],
             end_definitions: &[[Plane; 3]]| {
                Ok((*from == start
                    && *to == detour_point
                    && start_definitions == [start_definition.clone()]
                    && end_definitions == [detour_definition.clone()])
                    || (*from == detour_point
                        && *to == end
                        && start_definitions == [detour_definition.clone()]
                        && end_definitions == [end_definition.clone()]))
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
                &start_definition,
                &end_definition,
                |from, to, start_definitions, end_definitions| {
                    probe_reaches_adjacent_cell_with_definitions_budget_impl(
                        from,
                        to,
                        &[],
                        start_definitions,
                        end_definitions,
                        1,
                        &mut trace_without_detours,
                        &mut detours_for,
                    )
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_plane_replacement_reachability_surfaces_uncertified_intermediate_orderings() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = [
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(2, r(0)),
        ];

        let err = plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
            &start_definition,
            &end_definition,
            |_from, _to, _start_definitions, _end_definitions| Ok(false),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn definition_pair_reachability_backtracks_after_uncertified_pair() {
        let start_unknown = axis_plane_definition(&p(0, 0, 0));
        let start_ok = axis_plane_definition(&p(1, 0, 0));
        let end = axis_plane_definition(&p(2, 0, 0));

        assert!(definition_pair_reachability_backtracking_unknown(
            &[start_unknown.clone(), start_ok.clone()],
            std::slice::from_ref(&end),
            |start_definition, end_definition| {
                if start_definition == &start_unknown && end_definition == &end {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(start_definition == &start_ok && end_definition == &end)
                }
            },
        )
        .unwrap());
    }

    #[test]
    fn definition_pair_reachability_reports_unknown_if_all_pairs_are_uncertified() {
        let start = axis_plane_definition(&p(0, 0, 0));
        let end = axis_plane_definition(&p(1, 0, 0));

        let err = definition_pair_reachability_backtracking_unknown(
            std::slice::from_ref(&start),
            std::slice::from_ref(&end),
            |_start_definition, _end_definition| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn plane_replacement_step_detours_preserve_intermediate_definitions() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-1)),
            Plane::axis_aligned(1, r(0)),
            Plane::axis_aligned(2, r(0)),
        ];
        let expected_start_definitions = vec![start_definition.clone()];
        let expected_end_definitions = vec![end_definition.clone()];
        let expected_start = p(0, 0, 0);
        let expected_end = p(1, 0, 0);

        let winding = trace_plane_replacement_path_with_step_detours_impl(
            &start_definition,
            &end_definition,
            &[7],
            &[],
            |current, next, attempt, _polygons, current_definitions, next_definitions| {
                if *current == expected_start
                    && *next == expected_end
                    && current_definitions == expected_start_definitions.as_slice()
                    && next_definitions == expected_end_definitions.as_slice()
                {
                    Ok(Some(attempt.to_vec()))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert_eq!(winding, vec![7]);
    }

    #[test]
    fn recursive_detour_budget_retries_detour_legs() {
        let start = p(0, 0, 0);
        let inner = p(1, 0, 0);
        let outer = p(2, 0, 0);
        let end = p(3, 0, 0);
        let outer_target = DetourTarget {
            point: outer.clone(),
            definitions: vec![axis_plane_definition(&outer)],
        };
        let inner_target = DetourTarget {
            point: inner.clone(),
            definitions: vec![axis_plane_definition(&inner)],
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             winding: &[i32],
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                if (*from == start && *to == inner)
                    || (*from == inner && *to == outer)
                    || (*from == outer && *to == end)
                {
                    Ok(Some(winding.to_vec()))
                } else {
                    Ok(None)
                }
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![outer_target.clone()])
            } else if *from == start && *to == outer {
                Ok(vec![inner_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert_eq!(
            trace_segment_from_definitions_with_budget_impl(
                &start,
                &end,
                &[0],
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            ),
            Err(HypermeshError::UnknownClassification)
        );

        let with_nested = trace_segment_from_definitions_with_budget_impl(
            &start,
            &end,
            &[0],
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            2,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap();
        assert_eq!(with_nested, vec![0]);
    }

    #[test]
    fn probe_fallback_retries_axis_start_after_retained_definitions_fail() {
        let ref_point = p(0, 0, 0);
        let invalid_ref_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(2)),
        ];
        let valid_probe_definition = [
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(1, r(1)),
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
        ];
        let probe = ProbePoint {
            point: p(2, 1, 0),
            side: Classification::Positive,
            planes: vec![valid_probe_definition],
        };
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];

        assert_eq!(
            trace_segment(&ref_point, &probe.point, &[0], &[wall.clone()]),
            Err(HypermeshError::UnknownClassification)
        );

        let winding =
            trace_probe_winding(&ref_point, &[invalid_ref_definition], &probe, &[0], &[wall])
                .unwrap();

        assert_eq!(winding, Some(vec![0]));
    }
}
